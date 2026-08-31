#!/usr/bin/env python3
"""按固定提交获取 ROS 2 FAST-LIO2，不把第三方源码提交到本仓库。"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import tempfile
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY = "https://github.com/Ericsii/FAST_LIO_ROS2.git"
BRANCH = "ros2"
COMMIT = "2fffc570a25d0df172720bac034fbdb6a13d2162"


def run(command: list[str], cwd: Path | None = None) -> str:
    result = subprocess.run(command, cwd=cwd, capture_output=True, text=True, check=False)
    if result.returncode:
        details = (result.stderr or result.stdout).strip()
        raise SystemExit(f"命令失败 ({' '.join(command)})：\n{details}")
    return result.stdout.strip()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--destination", type=Path, default=PROJECT_ROOT / "src/fast_lio", help="检出目录"
    )
    return parser.parse_args()


def checkout_is_ready(destination: Path) -> bool:
    if not (destination / ".git").exists():
        return False
    if run(["git", "rev-parse", "HEAD"], destination) != COMMIT:
        return False
    status = run(["git", "submodule", "status", "--recursive"], destination)
    return all(line and line[0] not in "-+U" for line in status.splitlines())


def main() -> int:
    args = parse_args()
    destination = args.destination.expanduser().resolve()
    if destination.exists():
        if checkout_is_ready(destination):
            print(f"FAST-LIO2 已就绪：{destination} @ {COMMIT}")
            return 0
        raise SystemExit(f"目标已存在但不是锁定版本，拒绝覆盖：{destination}")

    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="fast-lio-fetch-", dir=destination.parent) as temporary:
        checkout = Path(temporary) / "fast_lio"
        run(["git", "clone", "--quiet", "--recursive", "--branch", BRANCH, REPOSITORY, str(checkout)])
        run(["git", "checkout", "--quiet", "--detach", COMMIT], checkout)
        run(["git", "submodule", "update", "--init", "--recursive"], checkout)
        if not checkout_is_ready(checkout):
            raise SystemExit("FAST-LIO2 检出校验失败")
        shutil.move(str(checkout), destination)

    print(f"已获取 FAST-LIO2：{destination} @ {COMMIT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
