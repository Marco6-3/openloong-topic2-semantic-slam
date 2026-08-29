#!/usr/bin/env python3
"""Export the pinned lightweight segmentation model to fixed-shape ONNX."""

import argparse
import os
from pathlib import Path

import onnx
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()

    args.destination.parent.mkdir(parents=True, exist_ok=True)
    exported = Path(
        YOLO(str(args.source)).export(
            format="onnx",
            imgsz=320,
            opset=17,
            simplify=False,
            dynamic=False,
            batch=1,
            device="cpu",
        )
    )
    model = onnx.load(str(exported))
    onnx.checker.check_model(model)
    os.replace(exported, args.destination)
    print(f"[model] exported and checked {args.destination}")


if __name__ == "__main__":
    main()
