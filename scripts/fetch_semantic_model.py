#!/usr/bin/env python3
"""按固定 URL 和 SHA-256 下载 Open3D-ML RandLA-Net SemanticKITTI 权重。"""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import urllib.request


MODEL_URL = (
    "https://storage.googleapis.com/open3d-releases/model-zoo/"
    "randlanet_semantickitti_202201071330utc.pth"
)
MODEL_SHA256 = "8929a19da311a031245f70cfeaee8221ed50f15c4b932ec34434c9daaf75750a"
DEFAULT_OUTPUT = Path("data/models/randlanet_semantickitti_202201071330utc.pth")


def digest(path: Path) -> str:
    checksum = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    output = parse_args().output.expanduser().resolve()
    if output.is_file():
        actual = digest(output)
        if actual != MODEL_SHA256:
            raise SystemExit(f"已有权重校验失败：expected={MODEL_SHA256}, actual={actual}")
        print(f"模型权重已存在且校验通过：{output}")
        return 0
    if output.exists():
        raise SystemExit(f"输出不是普通文件：{output}")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_name(f".{output.name}.{os.getpid()}.part")
    try:
        print(f"下载：{MODEL_URL}")
        urllib.request.urlretrieve(MODEL_URL, temporary)
        actual = digest(temporary)
        if actual != MODEL_SHA256:
            raise SystemExit(f"下载权重校验失败：expected={MODEL_SHA256}, actual={actual}")
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"模型权重已下载并校验：{output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
