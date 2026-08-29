#!/usr/bin/env python3
"""按相对起点和时长截取 ROS1 bag，保留原始序列化消息。"""

from __future__ import annotations

import argparse
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="输入 ROS1 .bag")
    parser.add_argument("output", type=Path, help="输出 ROS1 .bag（必须不存在）")
    parser.add_argument("--start", type=float, default=0.0, help="相对 bag 起点秒数")
    parser.add_argument("--duration", type=float, default=60.0, help="截取时长（秒）")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    target = args.output.expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"输入文件不存在：{source}")
    if target.exists():
        raise SystemExit(f"为避免覆盖，输出必须不存在：{target}")
    if args.start < 0 or args.duration <= 0:
        raise SystemExit("--start 必须非负，--duration 必须大于 0")
    target.parent.mkdir(parents=True, exist_ok=True)

    written = 0
    with AnyReader([source]) as reader, Writer(target) as writer:
        start_ns = reader.start_time + round(args.start * 1e9)
        stop_ns = start_ns + round(args.duration * 1e9)
        output_connections = {}
        for connection in reader.connections:
            output_connections[connection.id] = writer.add_connection(
                connection.topic,
                connection.msgtype,
                msgdef=connection.msgdef.data,
                md5sum=connection.digest,
                callerid=connection.ext.callerid,
                latching=connection.ext.latching,
            )
        for connection, timestamp, rawdata in reader.messages(start=start_ns, stop=stop_ns):
            writer.write(output_connections[connection.id], timestamp, rawdata)
            written += 1

    print(f"已写入 {written} 条消息：{target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
