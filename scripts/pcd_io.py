"""小型、无额外依赖的 PCD v0.7 二进制读写工具。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class PCDMetadata:
    fields: tuple[str, ...]
    sizes: tuple[int, ...]
    types: tuple[str, ...]
    counts: tuple[int, ...]
    width: int
    height: int
    points: int
    data: str
    payload_offset: int
    dtype: np.dtype


def numpy_type(type_code: str, size: int) -> str:
    prefixes = {"F": "f", "I": "i", "U": "u"}
    if type_code not in prefixes or size not in (1, 2, 4, 8):
        raise ValueError(f"不支持的 PCD 字段类型：{type_code}{size}")
    if type_code == "F" and size not in (4, 8):
        raise ValueError(f"不支持的 PCD 浮点大小：{size}")
    return f"<{prefixes[type_code]}{size}"


def parse_header(path: Path) -> PCDMetadata:
    values: dict[str, list[str]] = {}
    with path.open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD 缺少 DATA 行")
            try:
                text = line.decode("ascii").strip()
            except UnicodeDecodeError as exc:
                raise ValueError("PCD 头部不是 ASCII") from exc
            if not text or text.startswith("#"):
                continue
            parts = text.split()
            values[parts[0].upper()] = parts[1:]
            if parts[0].upper() == "DATA":
                offset = stream.tell()
                break

    required = ("FIELDS", "SIZE", "TYPE", "WIDTH", "HEIGHT", "POINTS", "DATA")
    missing = [name for name in required if name not in values]
    if missing:
        raise ValueError(f"PCD 头部缺少字段：{', '.join(missing)}")
    fields = tuple(values["FIELDS"])
    sizes = tuple(int(value) for value in values["SIZE"])
    types = tuple(values["TYPE"])
    counts = tuple(int(value) for value in values.get("COUNT", ["1"] * len(fields)))
    if not (len(fields) == len(sizes) == len(types) == len(counts)) or len(set(fields)) != len(fields):
        raise ValueError("PCD FIELDS/SIZE/TYPE/COUNT 数量不一致或字段重名")
    dtype_fields = []
    for name, size, type_code, count in zip(fields, sizes, types, counts, strict=True):
        if count < 1:
            raise ValueError("PCD COUNT 必须为正")
        base = numpy_type(type_code, size)
        dtype_fields.append((name, base) if count == 1 else (name, base, (count,)))
    dtype = np.dtype(dtype_fields, align=False)
    width, height, points = int(values["WIDTH"][0]), int(values["HEIGHT"][0]), int(values["POINTS"][0])
    if width < 0 or height < 0 or points < 0 or width * height != points:
        raise ValueError("PCD WIDTH * HEIGHT 与 POINTS 不一致")
    return PCDMetadata(
        fields, sizes, types, counts, width, height, points, values["DATA"][0].lower(), offset, dtype
    )


def read_pcd(path: Path) -> tuple[PCDMetadata, np.ndarray]:
    path = path.expanduser().resolve()
    metadata = parse_header(path)
    if metadata.data != "binary":
        raise ValueError(f"当前只支持 DATA binary，实际为 {metadata.data}")
    expected = metadata.payload_offset + metadata.points * metadata.dtype.itemsize
    if path.stat().st_size != expected:
        raise ValueError(f"PCD 文件大小不匹配：expected={expected}, actual={path.stat().st_size}")
    points = np.memmap(
        path, mode="r", dtype=metadata.dtype, offset=metadata.payload_offset, shape=(metadata.points,)
    )
    return metadata, points


def pcd_type(dtype: np.dtype) -> tuple[str, int]:
    base = dtype.base
    if base.kind == "f":
        return "F", base.itemsize
    if base.kind == "i":
        return "I", base.itemsize
    if base.kind == "u":
        return "U", base.itemsize
    raise ValueError(f"无法写入 PCD 的 NumPy 类型：{dtype}")


def write_pcd(path: Path, points: np.ndarray) -> None:
    if points.dtype.names is None:
        raise ValueError("PCD 输出必须是结构化 NumPy 数组")
    fields = list(points.dtype.names)
    sizes: list[int] = []
    types: list[str] = []
    counts: list[int] = []
    for name in fields:
        field_dtype = points.dtype.fields[name][0]
        type_code, size = pcd_type(field_dtype)
        sizes.append(size)
        types.append(type_code)
        counts.append(int(np.prod(field_dtype.shape)) if field_dtype.shape else 1)
    header = "\n".join(
        [
            "# .PCD v0.7 - Point Cloud Data file format",
            "VERSION 0.7",
            f"FIELDS {' '.join(fields)}",
            f"SIZE {' '.join(map(str, sizes))}",
            f"TYPE {' '.join(types)}",
            f"COUNT {' '.join(map(str, counts))}",
            f"WIDTH {len(points)}",
            "HEIGHT 1",
            "VIEWPOINT 0 0 0 1 0 0 0",
            f"POINTS {len(points)}",
            "DATA binary",
            "",
        ]
    ).encode("ascii")
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("xb") as stream:
        stream.write(header)
        stream.write(np.ascontiguousarray(points).tobytes())
