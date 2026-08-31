#!/usr/bin/env python3
"""将比赛 ROS1 bag 转为 ROS2 MCAP，并核对每个保留话题的消息数。"""

from __future__ import annotations

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

from rosbags.convert.commands import command as convert_command
from rosbags.highlevel import AnyReader


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="输入 ROS1 .bag")
    parser.add_argument("output", type=Path, help="输出 ROS2 bag 目录（必须不存在）")
    parser.add_argument("--metadata", type=Path, help="转换校验 JSON；默认放在输出目录旁")
    parser.add_argument("--keep-rosout", action="store_true", help="保留无关的 ROS1 /rosout")
    parser.add_argument("--compress", action="store_true", help="对 MCAP 使用 zstd 文件压缩")
    return parser.parse_args()


def topic_counts(path: Path, excluded: set[str] | None = None) -> dict[str, int]:
    excluded = excluded or set()
    with AnyReader([path]) as reader:
        return {
            connection.topic: int(connection.msgcount)
            for connection in reader.connections
            if connection.topic not in excluded
        }


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    target = args.output.expanduser().resolve()
    metadata = (
        args.metadata.expanduser().resolve()
        if args.metadata
        else target.with_name(f"{target.name}_conversion.json")
    )
    if not source.is_file():
        raise SystemExit(f"输入文件不存在：{source}")
    if target.exists() or metadata.exists():
        raise SystemExit("输出 bag 或转换摘要已存在，拒绝覆盖")

    excluded = set() if args.keep_rosout else {"/rosout"}
    source_counts = topic_counts(source, excluded)
    result = convert_command(
        srcs=[source],
        dst=target,
        dst_storage="mcap",
        compress="zstd" if args.compress else "none",
        exclude_topics=tuple(sorted(excluded)),
    )
    if result:
        raise SystemExit("rosbags 转换失败")

    target_counts = topic_counts(target)
    if source_counts != target_counts:
        raise SystemExit(f"转换后话题计数不一致：source={source_counts}, target={target_counts}")

    report = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input": str(source),
        "output": str(target),
        "storage": "mcap",
        "compression": "zstd" if args.compress else "none",
        "excluded_topics": sorted(excluded),
        "topic_counts": dict(sorted(target_counts.items())),
        "message_count": sum(target_counts.values()),
        "verified": True,
    }
    metadata.parent.mkdir(parents=True, exist_ok=True)
    metadata.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"转换完成并通过计数校验：{sum(target_counts.values()):,} 条消息 -> {target}")
    print(f"转换摘要：{metadata}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
