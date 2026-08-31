#!/usr/bin/env python3
"""把 FAST-LIO 输出 bag 的记录时间改为消息 Header 时间，供 /clock 正确回放。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path, help="输出 ROS2 MCAP 目录（必须不存在）")
    parser.add_argument(
        "--topics", nargs="+", default=["/Odometry", "/cloud_registered"], help="视频回放所需话题"
    )
    parser.add_argument("--metadata", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    target = args.output.expanduser().resolve()
    metadata = args.metadata.expanduser().resolve() if args.metadata else target.with_name(f"{target.name}_retime.json")
    if not source.exists():
        raise SystemExit(f"输入 bag 不存在：{source}")
    if target.exists() or metadata.exists():
        raise SystemExit("输出 bag 或摘要已存在，拒绝覆盖")
    selected = set(args.topics)
    counts = {topic: 0 for topic in selected}
    first_stamp: int | None = None
    last_stamp: int | None = None
    with AnyReader([source]) as reader, Writer(
        target, version=9, storage_plugin=StoragePlugin.MCAP
    ) as writer:
        connections = [connection for connection in reader.connections if connection.topic in selected]
        found = {connection.topic for connection in connections}
        if found != selected:
            raise SystemExit(f"输入 bag 缺少视频话题：{sorted(selected - found)}")
        outputs = {
            connection.id: writer.add_connection(
                connection.topic,
                connection.msgtype,
                msgdef=connection.msgdef.data,
                rihs01=connection.digest,
                serialization_format=connection.ext.serialization_format,
                offered_qos_profiles=connection.ext.offered_qos_profiles,
            )
            for connection in connections
        }
        for connection, _, rawdata in reader.messages(connections=connections):
            message = reader.deserialize(rawdata, connection.msgtype)
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            writer.write(outputs[connection.id], stamp, rawdata)
            counts[connection.topic] += 1
            first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
            last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)
    report = {
        "input": str(source),
        "output": str(target),
        "topic_counts": dict(sorted(counts.items())),
        "start_stamp_ns": first_stamp,
        "end_stamp_ns": last_stamp,
        "duration_seconds": (last_stamp - first_stamp) / 1e9 if first_stamp is not None else 0.0,
    }
    metadata.parent.mkdir(parents=True, exist_ok=True)
    metadata.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"回放 bag 已按 Header 重定时：{sum(counts.values()):,} 条消息 -> {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
