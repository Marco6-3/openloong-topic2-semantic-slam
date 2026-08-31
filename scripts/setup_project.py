#!/usr/bin/env python3
"""创建项目运行所需的本地目录，不下载或改写比赛数据。"""

from __future__ import annotations

from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOCAL_DIRECTORIES = (
    "data/raw",
    "data/samples",
    "data/intermediate",
    "data/outputs",
    "results/artifacts",
)


def main() -> int:
    for relative_path in LOCAL_DIRECTORIES:
        (PROJECT_ROOT / relative_path).mkdir(parents=True, exist_ok=True)

    candidates = (PROJECT_ROOT / "data/data.bag", PROJECT_ROOT / "data/raw/data.bag")
    bag = next((path for path in candidates if path.is_file()), None)

    print(f"项目目录已准备：{PROJECT_ROOT}")
    if bag:
        print(f"已发现数据包：{bag.relative_to(PROJECT_ROOT)} ({bag.stat().st_size:,} bytes)")
    else:
        print("未发现 data/data.bag 或 data/raw/data.bag；环境仍可正常使用。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
