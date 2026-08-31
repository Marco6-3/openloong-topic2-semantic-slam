#!/usr/bin/env python3
"""校验 OpenLoong 官方示例兼容的轨迹 YAML。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import yaml


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", type=Path)
    parser.add_argument("--output", "-o", type=Path)
    return parser.parse_args()


def validate(path: Path) -> dict[str, object]:
    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(document.get("path"), list):
        raise ValueError("顶层必须是含 path 列表的映射")
    rows = document["path"]
    if not rows or document.get("total_points") != len(rows):
        raise ValueError("path 为空或 total_points 与实际数量不一致")
    required = {"altitude", "latitude", "longitude", "stamp"}
    previous_stamp = -math.inf
    for index, row in enumerate(rows):
        if not isinstance(row, dict) or not required.issubset(row):
            raise ValueError(f"第 {index} 个轨迹点字段不完整")
        values = {name: float(row[name]) for name in required}
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError(f"第 {index} 个轨迹点含非有限值")
        if not -90 <= values["latitude"] <= 90 or not -180 <= values["longitude"] <= 180:
            raise ValueError(f"第 {index} 个轨迹点经纬度越界")
        if values["stamp"] <= previous_stamp:
            raise ValueError(f"第 {index} 个轨迹点时间戳不严格递增")
        previous_stamp = values["stamp"]
    return {
        "valid": True,
        "path": str(path.expanduser().resolve()),
        "total_points": len(rows),
        "start_stamp": float(rows[0]["stamp"]),
        "end_stamp": float(rows[-1]["stamp"]),
        "duration_seconds": float(rows[-1]["stamp"] - rows[0]["stamp"]),
    }


def main() -> int:
    args = parse_args()
    try:
        report = validate(args.path)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        raise SystemExit(f"轨迹校验失败：{exc}") from exc
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
