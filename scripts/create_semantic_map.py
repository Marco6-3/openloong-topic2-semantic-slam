#!/usr/bin/env python3
"""给几何 PCD 生成保守的高度语义基线（ground/low_object/structure/unknown）。"""

from __future__ import annotations

import argparse
import json
from collections import Counter
from pathlib import Path

import numpy as np
import yaml

if __package__:
    from .pcd_io import read_pcd, write_pcd
else:
    from pcd_io import read_pcd, write_pcd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="几何 PCD")
    parser.add_argument("output", type=Path, help="带 label/confidence/rgb 的 PCD（必须不存在）")
    parser.add_argument("--labels", type=Path, default=Path("config/semantic_labels.yaml"))
    parser.add_argument("--metadata", type=Path, help="分类摘要 JSON")
    parser.add_argument("--cell-size", type=float, default=0.5)
    parser.add_argument("--ground-height", type=float, default=0.25)
    parser.add_argument("--structure-height", type=float, default=1.8)
    parser.add_argument("--min-cell-points", type=int, default=2)
    return parser.parse_args()


def classify_geometry(
    xyz: np.ndarray,
    cell_size: float,
    ground_height: float,
    structure_height: float,
    min_cell_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    cells = np.floor(xyz[:, :2] / cell_size).astype(np.int64)
    _, inverse, counts = np.unique(cells, axis=0, return_inverse=True, return_counts=True)
    minimum_z = np.full(len(counts), np.inf)
    np.minimum.at(minimum_z, inverse, xyz[:, 2])
    height = xyz[:, 2] - minimum_z[inverse]
    supported = counts[inverse] >= min_cell_points
    labels = np.zeros(len(xyz), dtype=np.uint16)
    confidence = np.full(len(xyz), 0.1, dtype=np.float32)
    ground = supported & (height <= ground_height)
    low = supported & (height > ground_height) & (height < structure_height)
    structure = supported & (height >= structure_height)
    labels[ground], confidence[ground] = 1, 0.65
    labels[low], confidence[low] = 2, 0.35
    labels[structure], confidence[structure] = 3, 0.45
    return labels, confidence


def main() -> int:
    args = parse_args()
    source = args.input.expanduser().resolve()
    target = args.output.expanduser().resolve()
    labels_path = args.labels.expanduser().resolve()
    metadata_path = (
        args.metadata.expanduser().resolve()
        if args.metadata
        else target.with_suffix(".semantic.json")
    )
    if target.exists() or metadata_path.exists():
        raise SystemExit("语义 PCD 或摘要已存在，拒绝覆盖")
    if not labels_path.is_file() or args.cell_size <= 0 or args.min_cell_points < 1:
        raise SystemExit("标签文件不存在或分类参数无效")
    label_config = yaml.safe_load(labels_path.read_text(encoding="utf-8"))
    colors = {int(row["id"]): row["color_rgb"] for row in label_config["labels"]}
    names = {int(row["id"]): row["name"] for row in label_config["labels"]}

    pcd, points = read_pcd(source)
    if not {"x", "y", "z"}.issubset(pcd.fields):
        raise SystemExit("输入 PCD 缺少 x/y/z")
    xyz = np.column_stack([points[name] for name in ("x", "y", "z")]).astype(np.float32)
    finite = np.all(np.isfinite(xyz), axis=1)
    xyz = xyz[finite]
    labels, confidence = classify_geometry(
        xyz, args.cell_size, args.ground_height, args.structure_height, args.min_cell_points
    )
    intensity = (
        np.asarray(points["intensity"])[finite].astype(np.float32)
        if "intensity" in pcd.fields
        else np.zeros(len(xyz), dtype=np.float32)
    )
    rgb = np.asarray(
        [(colors[int(label)][0] << 16) | (colors[int(label)][1] << 8) | colors[int(label)][2] for label in labels],
        dtype=np.uint32,
    )
    output = np.empty(
        len(xyz),
        dtype=np.dtype(
            [("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
             ("label", "<u2"), ("confidence", "<f4"), ("rgb", "<u4")]
        ),
    )
    output["x"], output["y"], output["z"] = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    output["intensity"], output["label"], output["confidence"], output["rgb"] = (
        intensity, labels, confidence, rgb
    )
    write_pcd(target, output)
    counts = Counter(int(value) for value in labels)
    report = {
        "input": str(source),
        "output": str(target),
        "label_definition": str(labels_path),
        "method": "local_minimum_height_heuristic",
        "warning": "internal geometry baseline; not an official or learned semantic label set",
        "input_points": pcd.points,
        "output_points": len(output),
        "discarded_non_finite_points": int(np.count_nonzero(~finite)),
        "parameters": {
            "cell_size_m": args.cell_size,
            "ground_height_m": args.ground_height,
            "structure_height_m": args.structure_height,
            "min_cell_points": args.min_cell_points,
        },
        "label_counts": {names[key]: counts.get(key, 0) for key in sorted(names)},
    }
    metadata_path.parent.mkdir(parents=True, exist_ok=True)
    metadata_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"语义基线已生成：{len(output):,} 点 -> {target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
