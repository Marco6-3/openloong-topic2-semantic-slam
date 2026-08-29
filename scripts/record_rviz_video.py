#!/usr/bin/env python3
"""回放重定时后的 SLAM 话题，并用 GStreamer 录制不超过两分钟的 RViz WebM。"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from contextlib import ExitStack
from pathlib import Path
from typing import IO

from rosbags.highlevel import AnyReader
import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path, help="输出 .webm（必须不存在）")
    parser.add_argument("--rviz-config", type=Path, default=PROJECT_ROOT / "src/fast_lio/rviz/fastlio.rviz")
    parser.add_argument("--rate", type=float, default=6.0, help="回放倍率；652 秒以 6x 回放约 109 秒")
    parser.add_argument("--follow-frame", default="body", help="RViz 视角跟随的动态 TF；留空则使用原配置")
    parser.add_argument("--view-distance", type=float, default=45.0, help="跟随视角距离（米）")
    return parser.parse_args()


def start(command: list[str], log: IO[bytes]) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        command,
        cwd=PROJECT_ROOT,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def stop(process: subprocess.Popen[bytes] | None, interrupt: bool = False) -> None:
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT if interrupt else signal.SIGTERM)
    try:
        process.wait(timeout=20)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=5)


def find_rviz_window(config: Path, process: subprocess.Popen[bytes], timeout: float = 30.0) -> int:
    """等待 RViz 客户区出现，并返回它的 X11 window ID。"""
    if shutil.which("xwininfo") is None:
        raise RuntimeError("缺少 xwininfo，无法可靠定位 RViz 窗口")
    deadline = time.monotonic() + timeout
    pattern = re.compile(r'^\s*(0x[0-9a-fA-F]+)\s+".*": \("rviz2" "rviz2"\)')
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("RViz 在窗口出现前退出")
        tree = subprocess.run(
            ["xwininfo", "-root", "-tree"], capture_output=True, text=True, check=False
        ).stdout
        for line in tree.splitlines():
            if config.name not in line:
                continue
            match = pattern.match(line)
            if match:
                return int(match.group(1), 16)
        time.sleep(0.25)
    raise RuntimeError(f"30 秒内未发现 RViz 窗口：{config.name}")


def main() -> int:
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    output = args.output.expanduser().resolve()
    rviz_config = args.rviz_config.expanduser().resolve()
    if not bag.exists() or not rviz_config.is_file():
        raise SystemExit("视频 bag 或 RViz 配置不存在")
    if output.exists():
        raise SystemExit(f"视频已存在，拒绝覆盖：{output}")
    if args.rate <= 0 or args.view_distance <= 0:
        raise SystemExit("--rate 和 --view-distance 必须大于 0")
    if not os.environ.get("DISPLAY"):
        raise SystemExit("当前没有 DISPLAY，无法录制 RViz")
    if shutil.which("gst-launch-1.0") is None:
        raise SystemExit("缺少 gst-launch-1.0")
    # Pixi 的 GStreamer 核心优先于系统命令，但 ximagesrc 通常由 Ubuntu 插件提供。
    system_plugins = Path("/usr/lib/x86_64-linux-gnu/gstreamer-1.0")
    if (
        subprocess.run(["gst-inspect-1.0", "ximagesrc"], capture_output=True).returncode
        and system_plugins.is_dir()
    ):
        existing = os.environ.get("GST_PLUGIN_PATH", "")
        os.environ["GST_PLUGIN_PATH"] = f"{existing}:{system_plugins}" if existing else str(system_plugins)
    for plugin in ("ximagesrc", "vp8enc", "webmmux"):
        if subprocess.run(["gst-inspect-1.0", plugin], capture_output=True).returncode:
            raise SystemExit(f"缺少 GStreamer 插件：{plugin}")
    with AnyReader([bag]) as reader:
        duration = (reader.end_time - reader.start_time) / 1e9
    projected = duration / args.rate
    if projected > 118:
        raise SystemExit(f"预计视频 {projected:.1f} 秒，超过 118 秒安全上限")

    output.parent.mkdir(parents=True, exist_ok=True)
    generated_rviz_config = output.with_suffix(".rviz")
    if generated_rviz_config.exists():
        raise SystemExit(f"生成的 RViz 配置已存在，拒绝覆盖：{generated_rviz_config}")
    rviz_document = yaml.safe_load(rviz_config.read_text(encoding="utf-8"))
    if args.follow_frame:
        current_view = rviz_document["Visualization Manager"]["Views"]["Current"]
        current_view["Target Frame"] = args.follow_frame
        current_view["Distance"] = args.view_distance
        current_view["Focal Point"] = {"X": 0.0, "Y": 0.0, "Z": 0.0}
    generated_rviz_config.write_text(
        yaml.safe_dump(rviz_document, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    rviz_command = [
        "ros2", "run", "rviz2", "rviz2", "-d", str(generated_rviz_config),
        "--ros-args", "-p", "use_sim_time:=true"
    ]
    play_command = ["ros2", "bag", "play", str(bag), "--clock", "--rate", str(args.rate)]
    tf_command = [sys.executable, str(PROJECT_ROOT / "scripts/odom_to_tf.py")]
    rviz = capture = tf_broadcaster = None
    try:
        with ExitStack() as stack:
            rviz_log = stack.enter_context(output.with_suffix(".rviz.log").open("wb"))
            capture_log = stack.enter_context(output.with_suffix(".capture.log").open("wb"))
            play_log = stack.enter_context(output.with_suffix(".play.log").open("wb"))
            tf_log = stack.enter_context(output.with_suffix(".tf.log").open("wb"))
            tf_broadcaster = start(tf_command, tf_log)
            rviz = start(rviz_command, rviz_log)
            rviz_xid = find_rviz_window(generated_rviz_config, rviz)
            time.sleep(2)
            if tf_broadcaster.poll() is not None:
                raise RuntimeError("RViz 或里程计 TF 广播器启动失败，请检查日志")
            capture_command = [
                "gst-launch-1.0", "-e", "ximagesrc", "use-damage=false", f"xid={rviz_xid}",
                "!", "video/x-raw,framerate=30/1", "!", "videoconvert", "!", "queue",
                "!", "vp8enc", "deadline=1", "cpu-used=8", "target-bitrate=3500000",
                "!", "webmmux", "!", "filesink", f"location={output}",
            ]
            capture = start(capture_command, capture_log)
            time.sleep(2)
            if capture.poll() is not None:
                raise RuntimeError("GStreamer 录屏启动失败，请检查 capture 日志")
            print(f"正在录制 RViz：数据 {duration:.1f}s，{args.rate:g}x，预计 {projected:.1f}s……", flush=True)
            played = subprocess.run(play_command, cwd=PROJECT_ROOT, stdout=play_log, stderr=subprocess.STDOUT)
            if played.returncode:
                raise RuntimeError("RViz 回放失败")
            time.sleep(2)
            stop(capture, interrupt=True)
            capture = None
            stop(rviz)
            rviz = None
            stop(tf_broadcaster)
            tf_broadcaster = None
    finally:
        stop(capture, interrupt=True)
        stop(rviz)
        stop(tf_broadcaster)
    if not output.is_file() or output.stat().st_size == 0:
        raise SystemExit("未生成有效视频")
    print(f"RViz 视频已生成：{output} ({output.stat().st_size / 1024**2:.1f} MiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
