#!/usr/bin/env python3
"""用 GStreamer 校验 RViz 视频流、时长、尺寸和帧率。"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", type=Path)
    parser.add_argument("--max-duration", type=float, default=120.0)
    parser.add_argument("--output", "-o", type=Path)
    return parser.parse_args()


def parse_duration(value: str) -> float:
    match = re.fullmatch(r"(\d+):(\d+):(\d+(?:\.\d+)?)", value.strip())
    if not match:
        raise ValueError(f"无法解析视频时长：{value}")
    hours, minutes, seconds = match.groups()
    return int(hours) * 3600 + int(minutes) * 60 + float(seconds)


def parse_discovery(text: str) -> dict[str, object]:
    def require(pattern: str, name: str) -> re.Match[str]:
        match = re.search(pattern, text, flags=re.MULTILINE)
        if not match:
            raise ValueError(f"媒体摘要缺少{name}")
        return match

    duration = parse_duration(require(r"^\s*Duration:\s*(\S+)\s*$", "时长").group(1))
    codec = require(r"^\s*video #\d+:\s*(.+?)\s*$", "视频流").group(1)
    width = int(require(r"^\s*Width:\s*(\d+)\s*$", "宽度").group(1))
    height = int(require(r"^\s*Height:\s*(\d+)\s*$", "高度").group(1))
    frame_rate = require(r"^\s*Frame rate:\s*(\d+/\d+)\s*$", "帧率").group(1)
    return {
        "duration_seconds": duration,
        "codec": codec,
        "width": width,
        "height": height,
        "frame_rate": frame_rate,
    }


def validate(path: Path, max_duration: float = 120.0) -> dict[str, object]:
    path = path.expanduser().resolve()
    if max_duration <= 0:
        raise ValueError("最大时长必须大于 0")
    if not path.is_file() or path.stat().st_size == 0:
        raise ValueError("视频不存在或为空")
    executable = shutil.which("gst-discoverer-1.0")
    if executable is None:
        raise ValueError("缺少 gst-discoverer-1.0")
    environment = os.environ.copy()
    environment["LC_ALL"] = "C"
    system_plugins = Path("/usr/lib/x86_64-linux-gnu/gstreamer-1.0")
    if system_plugins.is_dir():
        existing = environment.get("GST_PLUGIN_PATH", "")
        environment["GST_PLUGIN_PATH"] = (
            f"{existing}:{system_plugins}" if existing else str(system_plugins)
        )
    discovered = subprocess.run(
        [executable, str(path)], capture_output=True, text=True, check=False,
        env=environment,
    )
    output = f"{discovered.stdout}\n{discovered.stderr}"
    if discovered.returncode:
        raise ValueError(f"GStreamer 无法读取视频：{output.strip()}")
    media = parse_discovery(output)
    duration = float(media["duration_seconds"])
    if duration <= 0 or duration > max_duration:
        raise ValueError(f"视频时长 {duration:.3f}s 不在 (0, {max_duration:.3f}] 范围")
    return {
        "valid": True,
        "path": str(path),
        "size_bytes": path.stat().st_size,
        "max_duration_seconds": max_duration,
        **media,
    }


def main() -> int:
    args = parse_args()
    try:
        report = validate(args.video, args.max_duration)
    except (OSError, ValueError) as exc:
        raise SystemExit(f"视频校验失败：{exc}") from exc
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
