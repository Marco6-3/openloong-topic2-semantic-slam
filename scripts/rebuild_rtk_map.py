#!/usr/bin/env python3
"""按逐帧 RTK 校正重建 ENU 体素地图，并按跨帧观测次数过滤瞬态点。"""

from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader

if __package__:
    from .pcd_io import write_pcd
else:
    from pcd_io import write_pcd


POINT_FIELD_TYPES = {
    1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cloud_bag", type=Path, help="run_fast_lio.py --record-clouds 产生的 bag")
    parser.add_argument("trajectory_csv", type=Path, help="fuse_trajectory.py 产生的 trajectory_fused.csv")
    parser.add_argument("trajectory_metrics", type=Path, help="fuse_trajectory.py 产生的 JSON")
    parser.add_argument("output", type=Path, help="ENU PCD（必须不存在）")
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--voxel-size", type=float, default=0.2)
    parser.add_argument("--min-observations", type=int, default=2)
    parser.add_argument("--merge-every", type=int, default=100, help="每多少帧合并一次体素数组")
    parser.add_argument("--metadata", type=Path, help="重建摘要 JSON")
    return parser.parse_args()


def pointcloud_array(message: Any) -> np.ndarray:
    names, formats, offsets = [], [], []
    for field in message.fields:
        if field.datatype not in POINT_FIELD_TYPES:
            raise ValueError(f"不支持 PointCloud2 datatype={field.datatype}")
        names.append(str(field.name))
        base = np.dtype(POINT_FIELD_TYPES[field.datatype])
        formats.append(base if field.count == 1 else np.dtype((base, (int(field.count),))))
        offsets.append(int(field.offset))
    dtype = np.dtype(
        {"names": names, "formats": formats, "offsets": offsets, "itemsize": int(message.point_step)}
    )
    raw = np.asarray(message.data, dtype=np.uint8)
    count = int(message.width) * int(message.height)
    if raw.nbytes < count * int(message.point_step):
        raise ValueError("PointCloud2 data 小于 width*height*point_step")
    return np.frombuffer(raw, dtype=dtype, count=count)


def read_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    stamps, corrections = [], []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamps.append(int(row["stamp_ns"]))
            corrections.append(
                [
                    float(row["correction_east_m"]),
                    float(row["correction_north_m"]),
                    float(row["correction_up_m"]),
                ]
            )
    return np.asarray(stamps, dtype=np.int64), np.asarray(corrections, dtype=np.float64)


def interpolate_correction(stamps: np.ndarray, corrections: np.ndarray, stamp: int) -> np.ndarray:
    return np.asarray([np.interp(stamp, stamps, corrections[:, axis]) for axis in range(3)])


def aggregate_frame(points: np.ndarray, voxel_size: float) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    keys = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 4), dtype=np.float64)
    np.add.at(sums, inverse, points)
    counts = np.bincount(inverse).astype(np.uint64)
    observations = np.ones(len(unique), dtype=np.uint32)
    return unique, sums, counts, observations


def merge_voxels(
    batches: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    keys = np.concatenate([batch[0] for batch in batches])
    sums_input = np.concatenate([batch[1] for batch in batches])
    counts_input = np.concatenate([batch[2] for batch in batches])
    observations_input = np.concatenate([batch[3] for batch in batches])
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), 4), dtype=np.float64)
    counts = np.zeros(len(unique), dtype=np.uint64)
    observations = np.zeros(len(unique), dtype=np.uint32)
    np.add.at(sums, inverse, sums_input)
    np.add.at(counts, inverse, counts_input)
    np.add.at(observations, inverse, observations_input)
    return unique, sums, counts, observations


def main() -> int:
    args = parse_args()
    cloud_bag = args.cloud_bag.expanduser().resolve()
    trajectory_csv = args.trajectory_csv.expanduser().resolve()
    metrics_path = args.trajectory_metrics.expanduser().resolve()
    output = args.output.expanduser().resolve()
    metadata_path = (
        args.metadata.expanduser().resolve() if args.metadata else output.with_suffix(".rebuild.json")
    )
    if not cloud_bag.exists() or not trajectory_csv.is_file() or not metrics_path.is_file():
        raise SystemExit("点云 bag、融合轨迹或轨迹指标不存在")
    if output.exists() or metadata_path.exists():
        raise SystemExit("输出 PCD 或摘要已存在，拒绝覆盖")
    if args.voxel_size <= 0 or args.min_observations < 1 or args.merge_every < 1:
        raise SystemExit("体素参数无效")

    trajectory_stamps, corrections = read_trajectory(trajectory_csv)
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    transform = metrics["transform_local_to_enu"]
    rotation = np.asarray(transform["rotation_xy"], dtype=np.float64)
    translation = np.asarray(
        [*transform["translation_xy_m"], transform["translation_z_m"]], dtype=np.float64
    )

    aggregate: tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
    pending: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
    frame_count = input_point_count = finite_point_count = 0
    with AnyReader([cloud_bag]) as reader:
        connections = [connection for connection in reader.connections if connection.topic == args.topic]
        if not connections:
            raise SystemExit(f"点云 bag 中没有话题 {args.topic}")
        for connection, _, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            cloud = pointcloud_array(message)
            if not {"x", "y", "z"}.issubset(cloud.dtype.names or ()):
                raise SystemExit("配准点云缺少 x/y/z 字段")
            xyz = np.column_stack([cloud[name] for name in ("x", "y", "z")]).astype(np.float64)
            intensity = (
                np.asarray(cloud["intensity"], dtype=np.float64)
                if "intensity" in (cloud.dtype.names or ())
                else np.zeros(len(xyz))
            )
            input_point_count += len(xyz)
            finite = np.all(np.isfinite(xyz), axis=1) & np.isfinite(intensity)
            xyz, intensity = xyz[finite], intensity[finite]
            finite_point_count += len(xyz)
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            correction = interpolate_correction(trajectory_stamps, corrections, stamp)
            xyz[:, :2] = xyz[:, :2] @ rotation.T
            xyz += translation + correction
            pending.append(aggregate_frame(np.column_stack([xyz, intensity]), args.voxel_size))
            frame_count += 1
            if len(pending) >= args.merge_every:
                aggregate = merge_voxels(([aggregate] if aggregate is not None else []) + pending)
                pending.clear()
                print(f"已重建 {frame_count} 帧，当前 {len(aggregate[0]):,} 个体素", flush=True)
    if pending:
        aggregate = merge_voxels(([aggregate] if aggregate is not None else []) + pending)
    if aggregate is None:
        raise SystemExit("没有可重建的点云帧")

    _, sums, counts, observations = aggregate
    keep = observations >= args.min_observations
    centroids = sums[keep] / counts[keep, None]
    output_points = np.empty(
        len(centroids),
        dtype=[("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
               ("observation_count", "<u4")],
    )
    for index, name in enumerate(("x", "y", "z", "intensity")):
        output_points[name] = centroids[:, index]
    output_points["observation_count"] = observations[keep]
    write_pcd(output, output_points)
    report = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input_cloud_bag": str(cloud_bag),
        "trajectory_csv": str(trajectory_csv),
        "topic": args.topic,
        "frame_count": frame_count,
        "input_point_count": input_point_count,
        "finite_point_count": finite_point_count,
        "voxel_size_m": args.voxel_size,
        "voxel_count_before_observation_filter": len(observations),
        "min_observations": args.min_observations,
        "output_point_count": len(output_points),
        "filtered_transient_voxels": int(np.count_nonzero(~keep)),
        "coordinate_frame": "ENU at origin from RTK metadata",
        "output": str(output),
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"RTK 校正地图已生成：{len(output_points):,} 点 -> {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
