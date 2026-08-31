#!/usr/bin/env python3
"""用轻量 RandLA-Net 逐帧推理，并经位姿投影、多帧融合生成 ENU 语义 PCD。"""

from __future__ import annotations

import argparse
from collections import Counter, deque
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
import platform
import time
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader
from scipy.spatial import cKDTree
import torch
import yaml

if __package__:
    from .fetch_semantic_model import MODEL_SHA256, MODEL_URL, digest
    from .pcd_io import read_pcd, write_pcd
    from .randlanet_model import (
        NUM_CLASSES,
        NUM_POINTS,
        load_pretrained,
        predict_probabilities,
        prepare_cloud,
    )
    from .rebuild_rtk_map import pointcloud_array
else:
    from fetch_semantic_model import MODEL_SHA256, MODEL_URL, digest
    from pcd_io import read_pcd, write_pcd
    from randlanet_model import NUM_CLASSES, NUM_POINTS, load_pretrained, predict_probabilities, prepare_cloud
    from rebuild_rtk_map import pointcloud_array


DEFAULT_WEIGHTS = Path("data/models/randlanet_semantickitti_202201071330utc.pth")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cloud_bag", type=Path, help="FAST-LIO 输出点云 bag")
    parser.add_argument("trajectory_csv", type=Path, help="带 LIO 位姿和逐帧 RTK 校正的 CSV")
    parser.add_argument("trajectory_metrics", type=Path, help="局部坐标到 ENU 变换 JSON")
    parser.add_argument("geometry", type=Path, help="要写入语义字段的最终 ENU 几何 PCD")
    parser.add_argument("output", type=Path, help="模型语义 PCD（必须不存在）")
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--labels", type=Path, default=Path("config/semantic_labels.yaml"))
    parser.add_argument("--metadata", type=Path, help="推理与融合摘要 JSON")
    parser.add_argument("--topic", default="/cloud_registered")
    parser.add_argument("--device", default="auto", help="auto、cuda 或 cpu")
    parser.add_argument("--window-frames", type=int, default=40, help="在线因果滑窗帧数")
    parser.add_argument("--min-window-frames", type=int, default=10)
    parser.add_argument("--inference-stride", type=int, default=10, help="每多少帧推理一次")
    parser.add_argument("--num-points", type=int, default=NUM_POINTS)
    parser.add_argument("--model-grid-size", type=float, default=0.06)
    parser.add_argument("--model-radius", type=float, default=50.0)
    parser.add_argument("--map-voxel-size", type=float, default=0.2)
    parser.add_argument("--confidence-threshold", type=float, default=0.35)
    parser.add_argument("--nearest-radius", type=float, default=0.35)
    parser.add_argument("--merge-every", type=int, default=25)
    parser.add_argument("--seed", type=int, default=20260829)
    parser.add_argument("--max-frames", type=int, help="仅调试：最多读取多少帧")
    return parser.parse_args()


def read_trajectory(path: Path) -> dict[str, np.ndarray]:
    names = (
        "local_x_m", "local_y_m", "local_z_m", "local_qx", "local_qy", "local_qz",
        "local_qw", "correction_east_m", "correction_north_m", "correction_up_m",
    )
    stamps: list[int] = []
    values: dict[str, list[float]] = {name: [] for name in names}
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamps.append(int(row["stamp_ns"]))
            for name in names:
                values[name].append(float(row[name]))
    if not stamps:
        raise ValueError("融合轨迹为空")
    output = {name: np.asarray(rows, dtype=np.float64) for name, rows in values.items()}
    output["stamp_ns"] = np.asarray(stamps, dtype=np.int64)
    return output


def nearest_trajectory_index(stamps: np.ndarray, stamp: int) -> int:
    index = int(np.searchsorted(stamps, stamp))
    candidates = [candidate for candidate in (index - 1, index) if 0 <= candidate < len(stamps)]
    return min(candidates, key=lambda candidate: abs(int(stamps[candidate]) - stamp))


def quaternion_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    norm = x * x + y * y + z * z + w * w
    if norm <= np.finfo(float).eps:
        raise ValueError("轨迹中存在零长度四元数")
    scale = 2.0 / norm
    return np.asarray(
        [
            [1 - scale * (y * y + z * z), scale * (x * y - z * w), scale * (x * z + y * w)],
            [scale * (x * y + z * w), 1 - scale * (x * x + z * z), scale * (y * z - x * w)],
            [scale * (x * z - y * w), scale * (y * z + x * w), 1 - scale * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def grid_centroids(
    model_points: np.ndarray, enu_points: np.ndarray, grid_size: float
) -> tuple[np.ndarray, np.ndarray]:
    keys = np.floor(model_points / grid_size).astype(np.int64)
    _, inverse = np.unique(keys, axis=0, return_inverse=True)
    count = np.bincount(inverse).astype(np.float64)
    model_sums = np.zeros((len(count), 3), dtype=np.float64)
    enu_sums = np.zeros((len(count), 3), dtype=np.float64)
    np.add.at(model_sums, inverse, model_points)
    np.add.at(enu_sums, inverse, enu_points)
    return (model_sums / count[:, None]).astype(np.float32), (enu_sums / count[:, None]).astype(np.float32)


def aggregate_prediction(
    enu_points: np.ndarray, probabilities: np.ndarray, voxel_size: float
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    keys = np.floor(enu_points / voxel_size).astype(np.int64)
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), NUM_CLASSES), dtype=np.float32)
    point_counts = np.bincount(inverse).astype(np.float32)
    np.add.at(sums, inverse, probabilities)
    sums /= point_counts[:, None]
    return unique, sums, np.ones(len(unique), dtype=np.uint16)


def merge_predictions(
    batches: list[tuple[np.ndarray, np.ndarray, np.ndarray]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    keys = np.concatenate([batch[0] for batch in batches])
    probabilities = np.concatenate([batch[1] for batch in batches])
    observations = np.concatenate([batch[2] for batch in batches])
    unique, inverse = np.unique(keys, axis=0, return_inverse=True)
    sums = np.zeros((len(unique), NUM_CLASSES), dtype=np.float32)
    merged_observations = np.zeros(len(unique), dtype=np.uint16)
    np.add.at(sums, inverse, probabilities)
    np.add.at(merged_observations, inverse, observations)
    return unique, sums, merged_observations


def structured_keys(keys: np.ndarray) -> np.ndarray:
    contiguous = np.ascontiguousarray(keys, dtype=np.int64)
    dtype = np.dtype([("x", "<i8"), ("y", "<i8"), ("z", "<i8")])
    return contiguous.view(dtype).reshape(-1)


def resolve_device(requested: str) -> torch.device:
    if requested == "auto":
        return torch.device("cuda" if torch.cuda.is_available() else "cpu")
    if requested not in {"cuda", "cpu"}:
        raise ValueError("--device 只能是 auto、cuda 或 cpu")
    if requested == "cuda" and not torch.cuda.is_available():
        raise ValueError("指定了 CUDA，但 PyTorch 未检测到可用 GPU")
    return torch.device(requested)


def main() -> int:
    args = parse_args()
    paths = {
        "cloud_bag": args.cloud_bag.expanduser().resolve(),
        "trajectory_csv": args.trajectory_csv.expanduser().resolve(),
        "trajectory_metrics": args.trajectory_metrics.expanduser().resolve(),
        "geometry": args.geometry.expanduser().resolve(),
        "weights": args.weights.expanduser().resolve(),
        "labels": args.labels.expanduser().resolve(),
        "output": args.output.expanduser().resolve(),
    }
    metadata_path = (
        args.metadata.expanduser().resolve()
        if args.metadata
        else paths["output"].with_suffix(".semantic.json")
    )
    required = ["cloud_bag", "trajectory_csv", "trajectory_metrics", "geometry", "weights", "labels"]
    missing = [str(paths[name]) for name in required if not paths[name].exists()]
    if missing:
        raise SystemExit(f"缺少输入：{missing}")
    if paths["output"].exists() or metadata_path.exists():
        raise SystemExit("语义 PCD 或摘要已存在，拒绝覆盖")
    if digest(paths["weights"]) != MODEL_SHA256:
        raise SystemExit("模型权重 SHA-256 与固定检查点不一致")
    if not (0 < args.confidence_threshold < 1) or args.nearest_radius < 0:
        raise SystemExit("置信度阈值或最近邻半径无效")
    positive = (
        args.window_frames, args.min_window_frames, args.inference_stride, args.num_points,
        args.model_grid_size, args.model_radius, args.map_voxel_size, args.merge_every,
    )
    if any(value <= 0 for value in positive) or args.min_window_frames > args.window_frames:
        raise SystemExit("滑窗、抽样、体素或融合参数无效")

    labels_config = yaml.safe_load(paths["labels"].read_text(encoding="utf-8"))
    label_rows = labels_config["labels"]
    names = {int(row["id"]): str(row["name"]) for row in label_rows}
    colors = {int(row["id"]): tuple(int(value) for value in row["color_rgb"]) for row in label_rows}
    if set(range(NUM_CLASSES + 1)) != set(names) or set(names) != set(colors):
        raise SystemExit("模型标签文件必须完整定义 0..19 类")

    trajectory = read_trajectory(paths["trajectory_csv"])
    metrics = json.loads(paths["trajectory_metrics"].read_text(encoding="utf-8"))
    transform = metrics["transform_local_to_enu"]
    rotation_xy = np.asarray(transform["rotation_xy"], dtype=np.float64)
    translation = np.asarray(
        [*transform["translation_xy_m"], transform["translation_z_m"]], dtype=np.float64
    )
    device = resolve_device(args.device)
    model = load_pretrained(paths["weights"], device)
    parameter_count = sum(parameter.numel() for parameter in model.parameters())

    window: deque[tuple[np.ndarray, np.ndarray]] = deque(maxlen=args.window_frames)
    pending: list[tuple[np.ndarray, np.ndarray, np.ndarray]] = []
    aggregate: tuple[np.ndarray, np.ndarray, np.ndarray] | None = None
    preparation_times: list[float] = []
    inference_times: list[float] = []
    raw_points_per_inference: list[int] = []
    grid_points_per_inference: list[int] = []
    frame_count = input_point_count = finite_point_count = inference_count = 0
    max_stamp_error_ns = 0
    last_inference_frame = -1
    total_started = time.perf_counter()

    def infer_window(frame_index: int, trajectory_index: int) -> None:
        nonlocal aggregate, inference_count, last_inference_frame
        local_world = np.concatenate([entry[0] for entry in window])
        enu = np.concatenate([entry[1] for entry in window])
        position = np.asarray(
            [trajectory[name][trajectory_index] for name in ("local_x_m", "local_y_m", "local_z_m")]
        )
        quaternion = [
            float(trajectory[name][trajectory_index])
            for name in ("local_qx", "local_qy", "local_qz", "local_qw")
        ]
        body_rotation = quaternion_matrix(*quaternion)
        model_points = (local_world - position) @ body_rotation
        radius = np.linalg.norm(model_points[:, :2], axis=1)
        keep = radius <= args.model_radius
        model_points, enu = model_points[keep], enu[keep]
        if not len(model_points):
            return
        raw_points_per_inference.append(len(model_points))
        model_points, enu = grid_centroids(model_points, enu, args.model_grid_size)
        grid_points_per_inference.append(len(model_points))
        start = time.perf_counter()
        prepared = prepare_cloud(
            model_points, device, num_points=args.num_points, seed=args.seed + frame_index
        )
        preparation_times.append(time.perf_counter() - start)
        if device.type == "cuda":
            torch.cuda.synchronize()
        start = time.perf_counter()
        probabilities = predict_probabilities(model, prepared)
        if device.type == "cuda":
            torch.cuda.synchronize()
        inference_times.append(time.perf_counter() - start)
        pending.append(
            aggregate_prediction(enu[prepared.selected_indices], probabilities, args.map_voxel_size)
        )
        inference_count += 1
        last_inference_frame = frame_index
        if len(pending) >= args.merge_every:
            aggregate = merge_predictions(([aggregate] if aggregate is not None else []) + pending)
            pending.clear()
        if inference_count % 25 == 0:
            voxel_count = (
                len(aggregate[0])
                if aggregate is not None
                else sum(len(batch[0]) for batch in pending)
            )
            print(
                f"已推理 {inference_count} 个滑窗 / 读取 {frame_index + 1} 帧，"
                f"语义体素约 {voxel_count:,}",
                flush=True,
            )

    last_trajectory_index = -1
    with AnyReader([paths["cloud_bag"]]) as reader:
        connections = [connection for connection in reader.connections if connection.topic == args.topic]
        if not connections:
            raise SystemExit(f"点云 bag 中没有话题 {args.topic}")
        for frame_index, (connection, _, rawdata) in enumerate(reader.messages(connections=connections)):
            if args.max_frames is not None and frame_index >= args.max_frames:
                break
            message = reader.deserialize(rawdata, connection.msgtype)
            cloud = pointcloud_array(message)
            if not {"x", "y", "z"}.issubset(cloud.dtype.names or ()):
                raise SystemExit("点云缺少 x/y/z 字段")
            local_world = np.column_stack([cloud[name] for name in ("x", "y", "z")]).astype(np.float64)
            input_point_count += len(local_world)
            finite = np.all(np.isfinite(local_world), axis=1)
            local_world = local_world[finite]
            finite_point_count += len(local_world)
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            trajectory_index = nearest_trajectory_index(trajectory["stamp_ns"], stamp)
            last_trajectory_index = trajectory_index
            stamp_error = abs(int(trajectory["stamp_ns"][trajectory_index]) - stamp)
            max_stamp_error_ns = max(max_stamp_error_ns, stamp_error)
            if stamp_error > 20_000_000:
                raise SystemExit(f"点云和轨迹时间差超过 20 ms：{stamp_error} ns")
            correction = np.asarray(
                [
                    trajectory["correction_east_m"][trajectory_index],
                    trajectory["correction_north_m"][trajectory_index],
                    trajectory["correction_up_m"][trajectory_index],
                ]
            )
            enu = local_world.copy()
            enu[:, :2] = enu[:, :2] @ rotation_xy.T
            enu += translation + correction
            window.append((local_world.astype(np.float32), enu.astype(np.float32)))
            frame_count += 1
            if (
                len(window) >= args.min_window_frames
                and (frame_index + 1) % args.inference_stride == 0
            ):
                infer_window(frame_index, trajectory_index)
    if frame_count == 0 or last_trajectory_index < 0:
        raise SystemExit("点云 bag 中没有可用帧")
    if len(window) >= args.min_window_frames and last_inference_frame != frame_count - 1:
        infer_window(frame_count - 1, last_trajectory_index)
    if pending:
        aggregate = merge_predictions(([aggregate] if aggregate is not None else []) + pending)
    if aggregate is None:
        raise SystemExit("没有产生语义推理结果")

    semantic_keys, probability_sums, semantic_observations = aggregate
    fused_probabilities = probability_sums / semantic_observations[:, None]
    _, geometry = read_pcd(paths["geometry"])
    geometry_xyz = np.column_stack([geometry[name] for name in ("x", "y", "z")]).astype(np.float64)
    geometry_keys = np.floor(geometry_xyz / args.map_voxel_size).astype(np.int64)
    semantic_structured = structured_keys(semantic_keys)
    geometry_structured = structured_keys(geometry_keys)
    insertion = np.searchsorted(semantic_structured, geometry_structured)
    exact = insertion < len(semantic_structured)
    exact[exact] &= semantic_structured[insertion[exact]] == geometry_structured[exact]
    output_probabilities = np.zeros((len(geometry), NUM_CLASSES), dtype=np.float32)
    output_observations = np.zeros(len(geometry), dtype=np.uint16)
    output_probabilities[exact] = fused_probabilities[insertion[exact]]
    output_observations[exact] = semantic_observations[insertion[exact]]
    nearest = np.zeros(len(geometry), dtype=bool)
    missing_indices = np.flatnonzero(~exact)
    if len(missing_indices) and args.nearest_radius > 0:
        semantic_centers = (semantic_keys.astype(np.float64) + 0.5) * args.map_voxel_size
        distance, neighbor = cKDTree(semantic_centers).query(
            geometry_xyz[missing_indices], k=1, distance_upper_bound=args.nearest_radius, workers=-1
        )
        valid = np.isfinite(distance) & (neighbor < len(semantic_keys))
        target = missing_indices[valid]
        output_probabilities[target] = fused_probabilities[neighbor[valid]]
        output_observations[target] = semantic_observations[neighbor[valid]]
        nearest[target] = True

    confidence = output_probabilities.max(axis=1)
    label = output_probabilities.argmax(axis=1).astype(np.uint16) + 1
    covered = exact | nearest
    label[~covered | (confidence < args.confidence_threshold)] = 0
    rgb = np.asarray(
        [
            (colors[int(value)][0] << 16)
            | (colors[int(value)][1] << 8)
            | colors[int(value)][2]
            for value in label
        ],
        dtype=np.uint32,
    )
    output_dtype = list(geometry.dtype.descr) + [("label", "<u2"), ("confidence", "<f4"), ("rgb", "<u4")]
    output = np.empty(len(geometry), dtype=output_dtype)
    for name in geometry.dtype.names or ():
        output[name] = geometry[name]
    output["label"] = label
    output["confidence"] = confidence
    output["rgb"] = rgb
    write_pcd(paths["output"], output)

    label_counts = Counter(int(value) for value in label)
    timing = lambda values, percentile: float(np.percentile(values, percentile)) if values else None
    report: dict[str, Any] = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "official_semantic_rule": (
            "no fixed class list; model-defined semantics and positions; "
            "model must suit edge computing"
        ),
        "model": {
            "architecture": "RandLA-Net",
            "training_dataset": "SemanticKITTI",
            "checkpoint_source": MODEL_URL,
            "checkpoint_sha256": MODEL_SHA256,
            "checkpoint_size_bytes": paths["weights"].stat().st_size,
            "parameter_count": parameter_count,
            "model_output_classes": NUM_CLASSES,
            "label_definition": str(paths["labels"]),
        },
        "runtime": {
            "device": str(device),
            "gpu": torch.cuda.get_device_name(device) if device.type == "cuda" else None,
            "torch_version": torch.__version__,
            "torch_cuda_version": torch.version.cuda,
            "python_platform": platform.platform(),
            "total_seconds": time.perf_counter() - total_started,
            "preparation_mean_ms": 1000 * float(np.mean(preparation_times)),
            "preparation_p95_ms": 1000 * timing(preparation_times, 95),
            "inference_mean_ms": 1000 * float(np.mean(inference_times)),
            "inference_p50_ms": 1000 * timing(inference_times, 50),
            "inference_p95_ms": 1000 * timing(inference_times, 95),
            "cuda_peak_memory_bytes": (
                int(torch.cuda.max_memory_allocated(device)) if device.type == "cuda" else None
            ),
        },
        "input": {
            "cloud_bag": str(paths["cloud_bag"]),
            "topic": args.topic,
            "frame_count": frame_count,
            "input_point_count": input_point_count,
            "finite_point_count": finite_point_count,
            "max_cloud_trajectory_stamp_error_ns": max_stamp_error_ns,
        },
        "inference": {
            "causal_window_frames": args.window_frames,
            "minimum_window_frames": args.min_window_frames,
            "stride_frames": args.inference_stride,
            "inference_count": inference_count,
            "num_points_per_inference": args.num_points,
            "model_grid_size_m": args.model_grid_size,
            "model_radius_m": args.model_radius,
            "raw_points_mean": float(np.mean(raw_points_per_inference)),
            "grid_points_mean": float(np.mean(grid_points_per_inference)),
        },
        "fusion": {
            "map_voxel_size_m": args.map_voxel_size,
            "semantic_voxel_count": len(semantic_keys),
            "confidence_threshold": args.confidence_threshold,
            "nearest_fill_radius_m": args.nearest_radius,
            "geometry_point_count": len(geometry),
            "exact_voxel_points": int(np.count_nonzero(exact)),
            "nearest_filled_points": int(np.count_nonzero(nearest)),
            "uncovered_points": int(np.count_nonzero(~covered)),
            "covered_fraction": float(np.count_nonzero(covered) / len(geometry)),
            "mean_observations_for_covered_points": float(np.mean(output_observations[covered])),
        },
        "labels": [
            {
                "id": identifier,
                "name": names[identifier],
                "point_count": label_counts.get(identifier, 0),
                "fraction": label_counts.get(identifier, 0) / len(label),
                "mean_confidence": float(np.mean(confidence[label == identifier]))
                if label_counts.get(identifier, 0)
                else None,
            }
            for identifier in range(NUM_CLASSES + 1)
        ],
        "coordinate_frame": "local ENU, identical point positions to geometry input",
        "output": str(paths["output"]),
        "caveat": (
            "SemanticKITTI-to-Livox domain shift is not ground-truth accuracy; "
            "confidence and spatial coverage are reported explicitly."
        ),
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(
        f"模型语义地图已生成：{len(output):,} 点，"
        f"覆盖 {report['fusion']['covered_fraction']:.2%} -> {paths['output']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
