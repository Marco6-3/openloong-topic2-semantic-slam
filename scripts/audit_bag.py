#!/usr/bin/env python3
"""只读审计 ROS1/ROS2 bag，并输出机器可读的 JSON 摘要。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
from rosbags.highlevel import AnyReader


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="ROS1 .bag 文件或 ROS2 bag 目录")
    parser.add_argument("--output", "-o", type=Path, help="JSON 输出路径；默认打印到标准输出")
    parser.add_argument("--samples", type=int, default=20, help="每个连接最多反序列化的均匀采样数")
    parser.add_argument("--sha256", action="store_true", help="计算输入文件 SHA-256（目录不支持）")
    return parser.parse_args()


def stamp_ns(value: Any) -> int | None:
    if value is None:
        return None
    if hasattr(value, "sec") and hasattr(value, "nanosec"):
        return int(value.sec) * 1_000_000_000 + int(value.nanosec)
    if hasattr(value, "secs") and hasattr(value, "nsecs"):
        return int(value.secs) * 1_000_000_000 + int(value.nsecs)
    return None


def scalar(value: Any) -> Any:
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def point_value(point: Any, name: str) -> Any:
    return point[name] if isinstance(point, np.void) else getattr(point, name)


def message_observation(msg: Any, bag_timestamp: int) -> dict[str, Any]:
    observation: dict[str, Any] = {"bag_timestamp_ns": bag_timestamp}
    header = getattr(msg, "header", None)
    if header is not None:
        observation["frame_id"] = str(getattr(header, "frame_id", ""))
        header_ns = stamp_ns(getattr(header, "stamp", None))
        observation["header_timestamp_ns"] = header_ns
        if header_ns is not None:
            observation["header_minus_bag_ms"] = round((header_ns - bag_timestamp) / 1e6, 6)

    for name in ("timebase", "point_num", "lidar_id", "rsvd"):
        if hasattr(msg, name):
            observation[name] = scalar(getattr(msg, name))

    if hasattr(msg, "data") and isinstance(getattr(msg, "data"), (int, float, np.generic)):
        observation["data"] = scalar(msg.data)
    status = getattr(msg, "status", None)
    if status is not None and hasattr(status, "status"):
        observation["navsat_status"] = int(status.status)
        observation["navsat_service"] = int(status.service)
    for name in ("latitude", "longitude", "altitude"):
        if hasattr(msg, name):
            observation[name] = float(getattr(msg, name))
    covariance = getattr(msg, "position_covariance", None)
    if covariance is not None:
        observation["position_covariance"] = [float(value) for value in covariance]
        observation["position_covariance_type"] = int(msg.position_covariance_type)
    twist = getattr(msg, "twist", None)
    if twist is not None:
        observation["linear_velocity"] = {
            axis: float(getattr(twist.linear, axis)) for axis in ("x", "y", "z")
        }

    fields = getattr(msg, "fields", None)
    if fields is not None:
        observation["point_fields"] = [
            {
                "name": str(field.name),
                "offset": int(field.offset),
                "datatype": int(field.datatype),
                "count": int(field.count),
            }
            for field in fields
        ]
        observation["point_step"] = int(getattr(msg, "point_step", 0))
        observation["row_step"] = int(getattr(msg, "row_step", 0))
        observation["width"] = int(getattr(msg, "width", 0))
        observation["height"] = int(getattr(msg, "height", 0))

    points = getattr(msg, "points", None)
    if points is not None:
        observation["points_container"] = type(points).__name__
        observation["points_length"] = len(points)
        if len(points):
            first = points[0]
            names = list(first.dtype.names or ()) if isinstance(first, np.void) else [
                name for name in ("offset_time", "x", "y", "z", "reflectivity", "tag", "line")
                if hasattr(first, name)
            ]
            observation["custom_point_fields"] = names
            if "offset_time" in names:
                offsets = np.asarray([point_value(p, "offset_time") for p in points], dtype=np.int64)
                observation["offset_time_min"] = int(offsets.min())
                observation["offset_time_max"] = int(offsets.max())
                observation["offset_time_monotonic"] = bool(np.all(offsets[1:] >= offsets[:-1]))
                observation["offset_time_negative_jumps"] = int(np.count_nonzero(np.diff(offsets) < 0))
    return observation


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    if not bag.exists():
        raise SystemExit(f"输入不存在：{bag}")
    if args.samples < 1:
        raise SystemExit("--samples 必须至少为 1")

    report: dict[str, Any] = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input": str(bag),
        "size_bytes": bag.stat().st_size if bag.is_file() else None,
    }
    if args.sha256:
        if not bag.is_file():
            raise SystemExit("--sha256 目前仅支持单文件 bag")
        report["sha256"] = sha256_file(bag)

    with AnyReader([bag]) as reader:
        start_ns = reader.start_time
        end_ns = reader.end_time
        report.update(
            {
                "start_ns": start_ns,
                "end_ns": end_ns,
                "duration_seconds": round((end_ns - start_ns) / 1e9, 9),
                "message_count": reader.message_count,
            }
        )

        per_topic: dict[str, dict[str, Any]] = {}
        topic_timestamps: dict[str, list[int]] = defaultdict(list)
        indices: dict[int, int] = defaultdict(int)
        sample_every: dict[int, int] = {}
        for connection in reader.connections:
            count = int(connection.msgcount)
            sample_every[connection.id] = max(1, math.ceil(count / args.samples))
            per_topic[connection.topic] = {
                "type": connection.msgtype,
                "count": count,
                "frequency_hz": round(count / ((end_ns - start_ns) / 1e9), 6),
                "connections": per_topic.get(connection.topic, {}).get("connections", 0) + 1,
                "message_definition": connection.msgdef.data,
                "observations": [],
                "deserialize_errors": [],
            }

        for connection, timestamp, rawdata in reader.messages():
            topic_timestamps[connection.topic].append(timestamp)
            index = indices[connection.id]
            indices[connection.id] += 1
            count = int(connection.msgcount)
            if index % sample_every[connection.id] != 0 and index != count - 1:
                continue
            topic = per_topic[connection.topic]
            try:
                msg = reader.deserialize(rawdata, connection.msgtype)
                topic["observations"].append(message_observation(msg, timestamp))
            except Exception as exc:  # 审计应继续报告其他话题
                topic["deserialize_errors"].append(f"message {index}: {type(exc).__name__}: {exc}")

        for topic_name, topic in per_topic.items():
            timestamps = np.asarray(topic_timestamps[topic_name], dtype=np.int64)
            if len(timestamps):
                topic["first_bag_timestamp_ns"] = int(timestamps[0])
                topic["last_bag_timestamp_ns"] = int(timestamps[-1])
            if len(timestamps) > 1:
                periods_ms = np.diff(timestamps) / 1e6
                span_seconds = (timestamps[-1] - timestamps[0]) / 1e9
                topic["observed_frequency_hz"] = round((len(timestamps) - 1) / span_seconds, 6)
                topic["period_ms"] = {
                    "min": round(float(periods_ms.min()), 6),
                    "median": round(float(np.median(periods_ms)), 6),
                    "p99": round(float(np.percentile(periods_ms, 99)), 6),
                    "max": round(float(periods_ms.max()), 6),
                }

        report["topics"] = dict(sorted(per_topic.items()))

    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    else:
        print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
