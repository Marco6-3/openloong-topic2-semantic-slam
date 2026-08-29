#!/usr/bin/env python3
"""校验二进制 PCD 的字段、点数、文件长度、有限值和坐标范围。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

if __package__:
    from .pcd_io import read_pcd
else:
    from pcd_io import read_pcd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("pcd", type=Path)
    parser.add_argument("--output", "-o", type=Path, help="可选的校验 JSON")
    parser.add_argument("--require-field", action="append", default=[], help="必须存在的额外字段")
    return parser.parse_args()


def validate(path: Path, required: tuple[str, ...] = ()) -> dict[str, object]:
    metadata, points = read_pcd(path)
    required_fields = ("x", "y", "z", *required)
    missing = [name for name in required_fields if name not in metadata.fields]
    if missing:
        raise ValueError(f"PCD 缺少必需字段：{', '.join(missing)}")
    if metadata.points == 0:
        raise ValueError("PCD 为空")
    xyz = np.column_stack([points[axis] for axis in ("x", "y", "z")]).astype(np.float64)
    finite = np.all(np.isfinite(xyz), axis=1)
    if not np.all(finite):
        raise ValueError(f"PCD 含 {np.count_nonzero(~finite)} 个非有限坐标点")
    bounds = {
        axis: {"min": float(xyz[:, index].min()), "max": float(xyz[:, index].max())}
        for index, axis in enumerate(("x", "y", "z"))
    }
    return {
        "valid": True,
        "path": str(path.expanduser().resolve()),
        "size_bytes": path.stat().st_size,
        "points": metadata.points,
        "fields": list(metadata.fields),
        "point_step_bytes": metadata.dtype.itemsize,
        "finite_xyz_points": int(np.count_nonzero(finite)),
        "bounds_m": bounds,
    }


def main() -> int:
    args = parse_args()
    try:
        report = validate(args.pcd, tuple(args.require_field))
    except (OSError, ValueError) as exc:
        raise SystemExit(f"PCD 校验失败：{exc}") from exc
    output = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output, encoding="utf-8")
    print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
