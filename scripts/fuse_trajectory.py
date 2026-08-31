#!/usr/bin/env python3
"""将 FAST-LIO2 轨迹与高质量 RTK 融合，输出 ENU CSV、官方格式 path.yaml 和指标。"""

from __future__ import annotations

import argparse
import csv
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import yaml
from rosbags.highlevel import AnyReader

if __package__:
    from .export_rtk_enu import enu_to_geodetic
else:  # 允许直接执行 `python scripts/fuse_trajectory.py`
    from export_rtk_enu import enu_to_geodetic


CSV_FIELDS = (
    "stamp_ns", "local_x_m", "local_y_m", "local_z_m", "local_qx", "local_qy",
    "local_qz", "local_qw", "aligned_east_m", "aligned_north_m", "aligned_up_m",
    "correction_east_m", "correction_north_m", "correction_up_m", "fused_east_m",
    "fused_north_m", "fused_up_m", "fused_qx", "fused_qy", "fused_qz", "fused_qw",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("lio_bag", type=Path, help="含 /Odometry 的 ROS2 bag")
    parser.add_argument("rtk_csv", type=Path, help="export_rtk_enu.py 生成的 CSV")
    parser.add_argument("rtk_metadata", type=Path, help="包含 ENU 原点的 JSON")
    parser.add_argument("output", type=Path, help="输出目录（必须不存在）")
    parser.add_argument("--topic", default="/Odometry", help="FAST-LIO 里程计话题")
    parser.add_argument("--max-association-gap", type=float, default=0.25, help="对齐时允许的 RTK 间隔秒数")
    parser.add_argument("--smooth-window", type=int, default=21, help="RTK 校正移动平均窗口（点数，奇数）")
    return parser.parse_args()


def read_lio_bag(path: Path, topic: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    stamps: list[int] = []
    positions: list[tuple[float, float, float]] = []
    quaternions: list[tuple[float, float, float, float]] = []
    with AnyReader([path]) as reader:
        connections = [connection for connection in reader.connections if connection.topic == topic]
        if not connections:
            raise SystemExit(f"里程计 bag 中没有话题 {topic}")
        for connection, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp = msg.header.stamp
            pose = msg.pose.pose
            stamps.append(int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec))
            positions.append((pose.position.x, pose.position.y, pose.position.z))
            quaternions.append(
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
            )
    stamp_array = np.asarray(stamps, dtype=np.int64)
    if len(stamp_array) < 3 or np.any(np.diff(stamp_array) <= 0):
        raise SystemExit("里程计时间戳数量不足或不严格递增")
    return stamp_array, np.asarray(positions, dtype=np.float64), np.asarray(quaternions)


def read_rtk_csv(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    stamps: list[int] = []
    positions: list[tuple[float, float, float]] = []
    weights: list[float] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamp_text = row.get("header_time_ns") or row["bag_time_ns"]
            stamps.append(int(stamp_text))
            positions.append((float(row["east_m"]), float(row["north_m"]), float(row["up_m"])))
            weights.append(float(row["horizontal_weight"]))
    stamp_array = np.asarray(stamps, dtype=np.int64)
    if len(stamp_array) < 3 or np.any(np.diff(stamp_array) <= 0):
        raise SystemExit("RTK 时间戳数量不足或不严格递增")
    return stamp_array, np.asarray(positions), np.asarray(weights)


def interpolate_positions(
    source_stamps: np.ndarray, source_positions: np.ndarray, target_stamps: np.ndarray
) -> np.ndarray:
    return np.column_stack(
        [np.interp(target_stamps, source_stamps, source_positions[:, axis]) for axis in range(3)]
    )


def association_mask(source_stamps: np.ndarray, target_stamps: np.ndarray, max_gap_ns: int) -> np.ndarray:
    right = np.searchsorted(source_stamps, target_stamps, side="left")
    valid = (right > 0) & (right < len(source_stamps))
    clipped = np.clip(right, 1, len(source_stamps) - 1)
    gaps = source_stamps[clipped] - source_stamps[clipped - 1]
    return valid & (gaps <= max_gap_ns)


def robust_planar_alignment(
    source_xy: np.ndarray, target_xy: np.ndarray, base_weights: np.ndarray | None = None
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """估计 target ~= R @ source + t，并用 Huber 权重降低离群点影响。"""
    if len(source_xy) < 3 or source_xy.shape != target_xy.shape:
        raise ValueError("平面对齐至少需要三个成对的二维点")
    weights = np.ones(len(source_xy)) if base_weights is None else np.asarray(base_weights).copy()
    weights /= np.median(weights)
    for _ in range(8):
        source_center = np.average(source_xy, axis=0, weights=weights)
        target_center = np.average(target_xy, axis=0, weights=weights)
        covariance = (source_xy - source_center).T @ (
            (target_xy - target_center) * weights[:, None]
        )
        left, _, right_t = np.linalg.svd(covariance)
        rotation = right_t.T @ left.T
        if np.linalg.det(rotation) < 0:
            right_t[-1] *= -1
            rotation = right_t.T @ left.T
        translation = target_center - rotation @ source_center
        residuals = np.linalg.norm(source_xy @ rotation.T + translation - target_xy, axis=1)
        median = np.median(residuals)
        scale = max(1.4826 * np.median(np.abs(residuals - median)), 0.02)
        huber = np.minimum(1.0, 1.5 * scale / np.maximum(residuals, 1e-12))
        weights = huber if base_weights is None else huber * np.asarray(base_weights) / np.median(base_weights)
    return rotation, translation, residuals


def smooth(values: np.ndarray, window: int) -> np.ndarray:
    if window <= 1:
        return values.copy()
    padding = window // 2
    padded = np.pad(values, ((padding, padding), (0, 0)), mode="edge")
    cumulative = np.vstack([np.zeros((1, values.shape[1])), np.cumsum(padded, axis=0)])
    return (cumulative[window:] - cumulative[:-window]) / window


def yaw_multiply(quaternions: np.ndarray, yaw: float) -> np.ndarray:
    half = yaw / 2.0
    yaw_quaternion = np.asarray([0.0, 0.0, math.sin(half), math.cos(half)])
    ax, ay, az, aw = np.moveaxis(np.broadcast_to(yaw_quaternion, quaternions.shape), 1, 0)
    bx, by, bz, bw = np.moveaxis(quaternions, 1, 0)
    result = np.column_stack(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )
    return result / np.linalg.norm(result, axis=1)[:, None]


def residual_metrics(residuals: np.ndarray) -> dict[str, float]:
    return {
        "rmse_m": float(np.sqrt(np.mean(np.square(residuals)))),
        "median_m": float(np.median(residuals)),
        "p95_m": float(np.percentile(residuals, 95)),
        "max_m": float(np.max(residuals)),
    }


def main() -> int:
    args = parse_args()
    lio_bag = args.lio_bag.expanduser().resolve()
    rtk_csv = args.rtk_csv.expanduser().resolve()
    metadata_path = args.rtk_metadata.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not lio_bag.exists() or not rtk_csv.is_file() or not metadata_path.is_file():
        raise SystemExit("LIO bag、RTK CSV 或 RTK metadata 不存在")
    if output.exists():
        raise SystemExit(f"输出目录已存在，拒绝覆盖：{output}")
    if args.max_association_gap <= 0 or args.smooth_window < 1 or args.smooth_window % 2 == 0:
        raise SystemExit("关联间隔必须为正，平滑窗口必须为正奇数")

    lio_stamps, local_positions, local_quaternions = read_lio_bag(lio_bag, args.topic)
    rtk_stamps, rtk_positions, rtk_weights = read_rtk_csv(rtk_csv)
    mask = association_mask(rtk_stamps, lio_stamps, round(args.max_association_gap * 1e9))
    if np.count_nonzero(mask) < 20:
        raise SystemExit("可用于 LIO/RTK 对齐的同步点不足 20 个")
    rtk_at_lio = interpolate_positions(rtk_stamps, rtk_positions, lio_stamps[mask])
    weights_at_lio = np.interp(lio_stamps[mask], rtk_stamps, rtk_weights)
    rotation, translation, horizontal_residuals = robust_planar_alignment(
        local_positions[mask, :2], rtk_at_lio[:, :2], weights_at_lio
    )
    aligned = np.empty_like(local_positions)
    aligned[:, :2] = local_positions[:, :2] @ rotation.T + translation
    z_offset = float(np.median(rtk_at_lio[:, 2] - local_positions[mask, 2]))
    aligned[:, 2] = local_positions[:, 2] + z_offset

    correction_mask = (rtk_stamps >= lio_stamps[0]) & (rtk_stamps <= lio_stamps[-1])
    correction_stamps = rtk_stamps[correction_mask]
    correction_targets = rtk_positions[correction_mask]
    lio_at_rtk = interpolate_positions(lio_stamps, aligned, correction_stamps)
    correction_samples = smooth(correction_targets - lio_at_rtk, args.smooth_window)
    corrections = interpolate_positions(correction_stamps, correction_samples, lio_stamps)
    fused = aligned + corrections

    yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    fused_quaternions = yaw_multiply(local_quaternions, yaw)
    fused_at_rtk = interpolate_positions(lio_stamps, fused, correction_stamps)
    fused_residuals = np.linalg.norm(fused_at_rtk - correction_targets, axis=1)

    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    origin = metadata["origin_wgs84"]
    origin_values = (origin["latitude"], origin["longitude"], origin["altitude"])
    path_rows = []
    for stamp, position in zip(lio_stamps, fused, strict=True):
        latitude, longitude, altitude = enu_to_geodetic(position, *origin_values)
        path_rows.append(
            {
                "altitude": float(altitude),
                "latitude": float(latitude),
                "longitude": float(longitude),
                "stamp": float(stamp / 1e9),
            }
        )

    output.mkdir(parents=True)
    csv_path = output / "trajectory_fused.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for index in range(len(lio_stamps)):
            values = [
                int(lio_stamps[index]), *local_positions[index], *local_quaternions[index],
                *aligned[index], *corrections[index], *fused[index], *fused_quaternions[index],
            ]
            writer.writerow(dict(zip(CSV_FIELDS, values, strict=True)))

    path_yaml = output / "path.yaml"
    path_yaml.write_text(
        yaml.safe_dump(
            {"path": path_rows, "total_points": len(path_rows)},
            allow_unicode=True,
            sort_keys=False,
            width=120,
        ),
        encoding="utf-8",
    )
    metrics = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "lio_count": len(lio_stamps),
        "rtk_count": len(rtk_stamps),
        "alignment_pair_count": int(np.count_nonzero(mask)),
        "correction_sample_count": len(correction_stamps),
        "transform_local_to_enu": {
            "yaw_degrees": math.degrees(yaw),
            "rotation_xy": rotation.tolist(),
            "translation_xy_m": translation.tolist(),
            "translation_z_m": z_offset,
        },
        "rigid_alignment_horizontal": residual_metrics(horizontal_residuals),
        "rtk_fused_3d": residual_metrics(fused_residuals),
        "smooth_window_points": args.smooth_window,
        "output": {"trajectory_csv": str(csv_path), "path_yaml": str(path_yaml)},
    }
    metrics_path = output / "trajectory_metrics.json"
    metrics_path.write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(
        f"已融合 {len(lio_stamps)} 个 LIO 位姿；刚体对齐水平 RMSE "
        f"{metrics['rigid_alignment_horizontal']['rmse_m']:.3f} m，"
        f"RTK 校正后 3D RMSE {metrics['rtk_fused_3d']['rmse_m']:.3f} m。"
    )
    print(f"官方格式轨迹：{path_yaml}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
