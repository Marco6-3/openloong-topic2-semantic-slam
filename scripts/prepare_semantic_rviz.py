#!/usr/bin/env python3
"""生成同时显示 SLAM 与彩色语义点云的 RViz 配置。"""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml

if __package__:
    from .record_rviz_video import PROJECT_ROOT, semantic_cloud_display
else:
    from record_rviz_video import PROJECT_ROOT, semantic_cloud_display


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base",
        type=Path,
        default=PROJECT_ROOT / "src/fast_lio/rviz/fastlio.rviz",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--follow-frame", default="body")
    parser.add_argument("--view-distance", type=float, default=45.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    source = args.base.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"基础 RViz 配置不存在：{source}")
    if output.exists():
        raise SystemExit(f"输出已存在，拒绝覆盖：{output}")
    document = yaml.safe_load(source.read_text(encoding="utf-8"))
    manager = document["Visualization Manager"]
    manager["Displays"] = [
        display
        for display in manager["Displays"]
        if display.get("Name") != "SemanticCloud (RandLA-Net RGB)"
    ]
    manager["Displays"].append(semantic_cloud_display())
    view = manager["Views"]["Current"]
    view["Target Frame"] = args.follow_frame
    view["Distance"] = args.view_distance
    view["Focal Point"] = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    print(f"语义录屏 RViz 配置已生成：{output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
