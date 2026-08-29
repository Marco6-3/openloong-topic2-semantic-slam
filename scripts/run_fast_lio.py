#!/usr/bin/env python3
"""在 ROS2 bag 上运行 FAST-LIO2，保存地图、轨迹话题、日志和性能摘要。"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
import time
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from typing import IO

from rosbags.highlevel import AnyReader


PROJECT_ROOT = Path(__file__).resolve().parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="已转换的 ROS2 bag 目录")
    parser.add_argument("output", type=Path, help="本次运行输出目录（必须不存在）")
    parser.add_argument(
        "--config", type=Path, default=PROJECT_ROOT / "config/fast_lio_mid360.yaml"
    )
    parser.add_argument("--rate", type=float, default=2.0, help="bag 播放倍率；实测 3x 会丢帧")
    parser.add_argument("--record-clouds", action="store_true", help="同时录制配准点云以便重建地图")
    parser.add_argument("--max-missing-frames", type=int, default=5, help="允许初始化阶段少于 LiDAR 的里程计帧数")
    return parser.parse_args()


def terminate_group(process: subprocess.Popen[bytes], timeout: float = 20.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


def group_usage(pgid: int) -> tuple[int, float]:
    rss_kib = 0
    cpu_ticks = 0
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="utf-8")
            fields = stat[stat.rfind(")") + 2 :].split()
            if int(fields[2]) != pgid:
                continue
            cpu_ticks += int(fields[11]) + int(fields[12])
            for line in (entry / "status").read_text(encoding="utf-8").splitlines():
                if line.startswith("VmRSS:"):
                    rss_kib += int(line.split()[1])
                    break
        except (FileNotFoundError, PermissionError, ProcessLookupError, ValueError):
            continue
    return rss_kib, cpu_ticks / os.sysconf("SC_CLK_TCK")


def monitor_group(pgid: int, stop: threading.Event, samples: dict[str, float]) -> None:
    while not stop.wait(0.5):
        rss_kib, cpu_seconds = group_usage(pgid)
        samples["peak_rss_kib"] = max(samples["peak_rss_kib"], float(rss_kib))
        samples["cpu_seconds"] = max(samples["cpu_seconds"], cpu_seconds)


def wait_for_map_service(timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = subprocess.run(
            ["ros2", "service", "list"], capture_output=True, text=True, timeout=5, check=False
        )
        if "/map_save" in result.stdout.splitlines():
            return
        time.sleep(0.5)
    raise RuntimeError("FAST-LIO2 未在限定时间内提供 /map_save 服务")


def directory_size(path: Path) -> int:
    return sum(item.stat().st_size for item in path.rglob("*") if item.is_file())


def start(command: list[str], log: IO[bytes]) -> subprocess.Popen[bytes]:
    return subprocess.Popen(
        command,
        cwd=PROJECT_ROOT,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )


def main() -> int:
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    output = args.output.expanduser().resolve()
    config = args.config.expanduser().resolve()
    if not bag.is_dir() or not config.is_file():
        raise SystemExit("ROS2 bag 目录或 FAST-LIO 配置不存在")
    if output.exists():
        raise SystemExit(f"输出目录已存在，拒绝覆盖：{output}")
    if args.rate <= 0:
        raise SystemExit("--rate 必须大于 0")
    if subprocess.run(["ros2", "pkg", "prefix", "fast_lio"], capture_output=True).returncode:
        raise SystemExit("未找到 fast_lio 包；请先获取依赖并执行 colcon build")

    with AnyReader([bag]) as reader:
        bag_duration = (reader.end_time - reader.start_time) / 1e9
        lidar_counts = [
            int(connection.msgcount)
            for connection in reader.connections
            if connection.topic == "/livox/lidar"
        ]
    if not lidar_counts:
        raise SystemExit("输入 bag 中没有 /livox/lidar")
    input_lidar_messages = sum(lidar_counts)

    output.mkdir(parents=True)
    map_path = output / "map_geometry.pcd"
    topics_path = output / "slam_topics"
    metrics_path = output / "run_metrics.json"
    topics = ["/Odometry"]
    if args.record_clouds:
        topics.extend(["/cloud_registered", "/cloud_registered_body"])

    lio_command = [
        "ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args",
        "--params-file", str(config), "-p", "use_sim_time:=true",
        "-p", f"map_file_path:={map_path}",
    ]
    record_command = ["ros2", "bag", "record", "-s", "mcap", "-o", str(topics_path), "--topics", *topics]
    play_command = ["ros2", "bag", "play", str(bag), "--clock", "--rate", str(args.rate)]

    lio: subprocess.Popen[bytes] | None = None
    recorder: subprocess.Popen[bytes] | None = None
    stop_monitor = threading.Event()
    usage = {"peak_rss_kib": 0.0, "cpu_seconds": 0.0}
    started = time.monotonic()
    try:
        with ExitStack() as stack:
            lio_log = stack.enter_context((output / "fast_lio.log").open("wb"))
            record_log = stack.enter_context((output / "record.log").open("wb"))
            play_log = stack.enter_context((output / "play.log").open("wb"))
            lio = start(lio_command, lio_log)
            wait_for_map_service(30)
            monitor = threading.Thread(
                target=monitor_group, args=(lio.pid, stop_monitor, usage), daemon=True
            )
            monitor.start()
            recorder = start(record_command, record_log)
            time.sleep(2)
            print(f"FAST-LIO2 已启动，以 {args.rate:g}x 播放 {bag_duration:.3f} 秒数据……")
            played = subprocess.run(play_command, cwd=PROJECT_ROOT, stdout=play_log, stderr=subprocess.STDOUT)
            if played.returncode:
                raise RuntimeError(f"ros2 bag play 失败，退出码 {played.returncode}")
            time.sleep(2)
            saved = subprocess.run(
                ["ros2", "service", "call", "/map_save", "std_srvs/srv/Trigger", "{}"],
                cwd=PROJECT_ROOT,
                capture_output=True,
                text=True,
                timeout=60,
                check=False,
            )
            if saved.returncode or "success=True" not in saved.stdout:
                raise RuntimeError(f"地图保存失败：{saved.stdout}{saved.stderr}")
            terminate_group(recorder)
            recorder = None
            terminate_group(lio)
            lio = None
            stop_monitor.set()
            monitor.join(timeout=2)
    finally:
        stop_monitor.set()
        if recorder is not None:
            terminate_group(recorder)
        if lio is not None:
            terminate_group(lio)

    wall_seconds = time.monotonic() - started
    if not map_path.is_file() or map_path.stat().st_size == 0 or not topics_path.is_dir():
        raise SystemExit("FAST-LIO2 未生成预期地图或轨迹 bag")
    with AnyReader([topics_path]) as reader:
        output_topic_counts = {
            connection.topic: int(connection.msgcount) for connection in reader.connections
        }
    odometry_count = output_topic_counts.get("/Odometry", 0)
    missing_frames = input_lidar_messages - odometry_count
    report = {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "input_bag": str(bag),
        "config": str(config),
        "playback_rate": args.rate,
        "bag_duration_seconds": bag_duration,
        "wall_seconds": wall_seconds,
        "effective_realtime_factor": bag_duration / wall_seconds,
        "fast_lio_peak_rss_mib": usage["peak_rss_kib"] / 1024.0,
        "fast_lio_cpu_seconds": usage["cpu_seconds"],
        "map_size_bytes": map_path.stat().st_size,
        "recorded_topics": topics,
        "input_lidar_messages": input_lidar_messages,
        "output_topic_counts": output_topic_counts,
        "missing_odometry_frames": missing_frames,
        "max_missing_frames": args.max_missing_frames,
        "recorded_bag_size_bytes": directory_size(topics_path),
        "commands": {"fast_lio": lio_command, "record": record_command, "play": play_command},
    }
    metrics_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if missing_frames < 0 or missing_frames > args.max_missing_frames:
        raise SystemExit(
            f"完整性门禁失败：LiDAR={input_lidar_messages}，Odometry={odometry_count}，"
            f"允许最多缺 {args.max_missing_frames} 帧"
        )
    if args.record_clouds and any(output_topic_counts.get(topic) != odometry_count for topic in topics[1:]):
        raise SystemExit(f"完整性门禁失败：输出话题数量不一致 {output_topic_counts}")
    print(f"运行完成：地图 {map_path}，轨迹话题 {topics_path}")
    print(f"性能摘要：{metrics_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
