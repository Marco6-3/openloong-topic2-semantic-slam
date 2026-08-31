#!/usr/bin/env python3
"""把最终模型语义逐帧投影回 SLAM 点云，并发布供 RViz 录屏的彩色 PointCloud2。"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
import yaml

if __package__:
    from .pcd_io import read_pcd
    from .rebuild_rtk_map import pointcloud_array
else:
    from pcd_io import read_pcd
    from rebuild_rtk_map import pointcloud_array


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SEMANTIC_DTYPE = np.dtype(
    {
        "names": ["x", "y", "z", "rgb", "label", "confidence"],
        "formats": ["<f4", "<f4", "<f4", "<f4", "<u2", "<f4"],
        "offsets": [0, 4, 8, 12, 16, 20],
        "itemsize": 24,
    }
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--semantic",
        type=Path,
        default=PROJECT_ROOT / "data/outputs/final/map_semantic.pcd",
    )
    parser.add_argument(
        "--trajectory",
        type=Path,
        default=PROJECT_ROOT / "data/outputs/fused/trajectory_fused.csv",
    )
    parser.add_argument(
        "--trajectory-metrics",
        type=Path,
        default=PROJECT_ROOT / "data/outputs/fused/trajectory_metrics.json",
    )
    parser.add_argument("--input-topic", default="/cloud_registered")
    parser.add_argument("--output-topic", default="/semantic_cloud")
    parser.add_argument("--labels", type=Path, default=PROJECT_ROOT / "config/semantic_labels.yaml")
    parser.add_argument("--nearest-radius", type=float, default=0.35)
    parser.add_argument("--point-stride", type=int, default=1, help="RViz 负载过高时按步长抽点")
    parser.add_argument("--log-every", type=int, default=100, help="每多少帧打印一次覆盖率")
    return parser.parse_args()


def read_trajectory(path: Path) -> tuple[np.ndarray, np.ndarray]:
    stamps: list[int] = []
    corrections: list[tuple[float, float, float]] = []
    with path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            stamps.append(int(row["stamp_ns"]))
            corrections.append(
                (
                    float(row["correction_east_m"]),
                    float(row["correction_north_m"]),
                    float(row["correction_up_m"]),
                )
            )
    if not stamps:
        raise ValueError("融合轨迹为空")
    return np.asarray(stamps, dtype=np.int64), np.asarray(corrections, dtype=np.float64)


def nearest_trajectory_index(stamps: np.ndarray, stamp: int) -> int:
    insertion = int(np.searchsorted(stamps, stamp))
    candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(stamps)]
    return min(candidates, key=lambda index: abs(int(stamps[index]) - stamp))


class SemanticProjector:
    def __init__(
        self,
        semantic_xyz: np.ndarray,
        labels: np.ndarray,
        confidence: np.ndarray,
        rgb: np.ndarray,
        trajectory_stamps: np.ndarray,
        corrections: np.ndarray,
        rotation_xy: np.ndarray,
        translation: np.ndarray,
        nearest_radius: float,
    ) -> None:
        if nearest_radius <= 0:
            raise ValueError("最近邻半径必须大于 0")
        if len(semantic_xyz) == 0 or not (
            len(semantic_xyz) == len(labels) == len(confidence) == len(rgb)
        ):
            raise ValueError("语义地图为空或字段长度不一致")
        if len(trajectory_stamps) != len(corrections):
            raise ValueError("轨迹时间戳与校正量长度不一致")
        self.tree = cKDTree(np.asarray(semantic_xyz, dtype=np.float64))
        self.labels = np.asarray(labels, dtype=np.uint16)
        self.confidence = np.asarray(confidence, dtype=np.float32)
        self.rgb = np.asarray(rgb, dtype=np.uint32)
        self.trajectory_stamps = np.asarray(trajectory_stamps, dtype=np.int64)
        self.corrections = np.asarray(corrections, dtype=np.float64)
        self.rotation_xy = np.asarray(rotation_xy, dtype=np.float64)
        self.translation = np.asarray(translation, dtype=np.float64)
        self.nearest_radius = nearest_radius

    def project(
        self, local_xyz: np.ndarray, stamp_ns: int
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, int]:
        trajectory_index = nearest_trajectory_index(self.trajectory_stamps, stamp_ns)
        stamp_error = abs(int(self.trajectory_stamps[trajectory_index]) - stamp_ns)
        enu = np.asarray(local_xyz, dtype=np.float64).copy()
        enu[:, :2] = enu[:, :2] @ self.rotation_xy.T
        enu += self.translation + self.corrections[trajectory_index]
        distance, neighbor = self.tree.query(
            enu, k=1, distance_upper_bound=self.nearest_radius, workers=1
        )
        valid = np.isfinite(distance) & (neighbor < len(self.labels))
        labels = np.zeros(len(enu), dtype=np.uint16)
        confidence = np.zeros(len(enu), dtype=np.float32)
        rgb = np.full(len(enu), 0x808080, dtype=np.uint32)
        labels[valid] = self.labels[neighbor[valid]]
        confidence[valid] = self.confidence[neighbor[valid]]
        rgb[valid] = self.rgb[neighbor[valid]]
        return labels, confidence, rgb, valid, stamp_error


def semantic_message(
    source: PointCloud2,
    xyz: np.ndarray,
    labels: np.ndarray,
    confidence: np.ndarray,
    rgb: np.ndarray,
) -> PointCloud2:
    points = np.empty(len(xyz), dtype=SEMANTIC_DTYPE)
    points["x"], points["y"], points["z"] = xyz.T.astype(np.float32)
    points["rgb"] = np.asarray(rgb, dtype="<u4").view("<f4")
    points["label"] = labels
    points["confidence"] = confidence
    output = PointCloud2()
    output.header = source.header
    output.height = 1
    output.width = len(points)
    output.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name="label", offset=16, datatype=PointField.UINT16, count=1),
        PointField(name="confidence", offset=20, datatype=PointField.FLOAT32, count=1),
    ]
    output.is_bigendian = False
    output.point_step = SEMANTIC_DTYPE.itemsize
    output.row_step = output.point_step * output.width
    output.data = points.tobytes()
    output.is_dense = bool(np.all(np.isfinite(xyz)))
    return output


class SemanticCloudPublisher(Node):
    def __init__(
        self, args: argparse.Namespace, projector: SemanticProjector, label_names: dict[int, str]
    ) -> None:
        super().__init__("semantic_cloud_publisher")
        self.projector = projector
        self.label_names = label_names
        self.point_stride = args.point_stride
        self.log_every = args.log_every
        self.frame_count = 0
        self.point_count = 0
        self.matched_count = 0
        self.max_stamp_error_ns = 0
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.publisher = self.create_publisher(PointCloud2, args.output_topic, qos)
        self.subscription = self.create_subscription(
            PointCloud2, args.input_topic, self.on_cloud, qos
        )
        self.get_logger().info(
            f"等待 {args.input_topic}；彩色语义点云将发布到 {args.output_topic}"
        )
        self.get_logger().info(
            f"RandLA-Net / SemanticKITTI：语义地图 {len(projector.labels):,} 点，"
            "输出字段 rgb + label + confidence"
        )

    def on_cloud(self, message: PointCloud2) -> None:
        cloud = pointcloud_array(message)
        xyz = np.column_stack([cloud[name] for name in ("x", "y", "z")]).astype(np.float64)
        finite = np.all(np.isfinite(xyz), axis=1)
        xyz = xyz[finite][:: self.point_stride]
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(
            message.header.stamp.nanosec
        )
        labels, confidence, rgb, matched, stamp_error = self.projector.project(xyz, stamp_ns)
        self.publisher.publish(semantic_message(message, xyz, labels, confidence, rgb))
        self.frame_count += 1
        self.point_count += len(xyz)
        self.matched_count += int(np.count_nonzero(matched))
        self.max_stamp_error_ns = max(self.max_stamp_error_ns, stamp_error)
        if self.frame_count == 1 or self.frame_count % self.log_every == 0:
            coverage = self.matched_count / self.point_count if self.point_count else 0.0
            unique, counts = np.unique(labels[matched], return_counts=True)
            leading = sorted(zip(counts.tolist(), unique.tolist()), reverse=True)[:4]
            label_summary = ", ".join(
                f"{self.label_names.get(label, 'unknown')}({label}):{count}"
                for count, label in leading
            ) or "none"
            mean_confidence = float(np.mean(confidence[matched])) if np.any(matched) else 0.0
            self.get_logger().info(
                f"语义帧 {self.frame_count}，累计点 {self.point_count:,}，"
                f"位置匹配覆盖 {coverage:.1%}，本帧平均置信度 {mean_confidence:.3f}，"
                f"主要标签 {label_summary}"
            )


def load_projector(args: argparse.Namespace) -> SemanticProjector:
    semantic_path = args.semantic.expanduser().resolve()
    trajectory_path = args.trajectory.expanduser().resolve()
    metrics_path = args.trajectory_metrics.expanduser().resolve()
    missing = [str(path) for path in (semantic_path, trajectory_path, metrics_path) if not path.is_file()]
    if missing:
        raise SystemExit(f"缺少语义可视化输入：{missing}")
    metadata, semantic = read_pcd(semantic_path)
    required = {"x", "y", "z", "label", "confidence", "rgb"}
    if not required.issubset(metadata.fields):
        raise SystemExit(f"语义 PCD 缺少字段：{sorted(required - set(metadata.fields))}")
    semantic_xyz = np.column_stack([semantic[name] for name in ("x", "y", "z")])
    stamps, corrections = read_trajectory(trajectory_path)
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    transform = metrics["transform_local_to_enu"]
    rotation_xy = np.asarray(transform["rotation_xy"], dtype=np.float64)
    translation = np.asarray(
        [*transform["translation_xy_m"], transform["translation_z_m"]], dtype=np.float64
    )
    return SemanticProjector(
        semantic_xyz,
        semantic["label"],
        semantic["confidence"],
        semantic["rgb"],
        stamps,
        corrections,
        rotation_xy,
        translation,
        args.nearest_radius,
    )


def main() -> int:
    args = parse_args()
    if args.point_stride <= 0 or args.log_every <= 0:
        raise SystemExit("--point-stride 和 --log-every 必须大于 0")
    projector = load_projector(args)
    labels_path = args.labels.expanduser().resolve()
    if not labels_path.is_file():
        raise SystemExit(f"缺少语义标签定义：{labels_path}")
    label_document = yaml.safe_load(labels_path.read_text(encoding="utf-8"))
    label_names = {int(row["id"]): str(row["name"]) for row in label_document["labels"]}
    rclpy.init()
    node = SemanticCloudPublisher(args, projector, label_names)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
