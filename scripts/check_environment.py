#!/usr/bin/env python3
"""检查项目要求的 ROS 2 Jazzy、编译工具及 Python 依赖。"""

from __future__ import annotations

import importlib.metadata
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any


def command_output(command: list[str]) -> str | None:
    if shutil.which(command[0]) is None:
        return None
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        return None
    output = (result.stdout or result.stderr).strip()
    return output.splitlines()[0] if output else "available"


def package_prefix(package: str) -> str | None:
    if shutil.which("ros2") is None:
        return None
    result = subprocess.run(
        ["ros2", "pkg", "prefix", package], capture_output=True, text=True, check=False
    )
    return result.stdout.strip() if result.returncode == 0 else None


def collect_report() -> dict[str, Any]:
    return {
        "platform": platform.platform(),
        "python": platform.python_version(),
        "ros_distro": os.environ.get("ROS_DISTRO"),
        "ros_version": os.environ.get("ROS_VERSION"),
        "ros_packages": {
            package: package_prefix(package) for package in ("rviz2", "rosbag2", "pcl_ros")
        },
        "tools": {
            "cmake": command_output(["cmake", "--version"]),
            "colcon": command_output(["colcon", "version-check"]),
            "compiler": command_output(["c++", "--version"]),
            "eigen": command_output(["pkg-config", "--modversion", "eigen3"]),
            "pcl": command_output(["pkg-config", "--modversion", "pcl_common"]),
        },
        "python_packages": {
            package: importlib.metadata.version(package)
            for package in ("numpy", "PyYAML", "rosbags")
        },
    }


def validate(report: dict[str, Any]) -> list[str]:
    errors: list[str] = []
    if report["ros_distro"] != "jazzy" or report["ros_version"] != "2":
        errors.append("当前环境不是 ROS 2 Jazzy")
    for package, prefix in report["ros_packages"].items():
        if prefix is None:
            errors.append(f"缺少 ROS 包：{package}")
    for tool, version in report["tools"].items():
        if version is None:
            errors.append(f"缺少工具或版本不可读：{tool}")
    return errors


def main() -> int:
    report = collect_report()
    errors = validate(report)
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if errors:
        print("\n环境检查失败：", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print("\n环境检查通过：ROS 2 Jazzy 与项目工具均可用。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
