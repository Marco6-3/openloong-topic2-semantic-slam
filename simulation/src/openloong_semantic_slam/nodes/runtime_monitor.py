#!/usr/bin/env python3
"""Compact Chinese status view for simultaneously recording SLAM and semantics."""

import json
import time

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String


class RuntimeMonitor:
    def __init__(self) -> None:
        self.semantic = {}
        self.path_poses = 0
        self.map_size = "等待地图"
        self.sim_time = None
        self.wall_time = None
        self.real_time_factor = 0.0
        rospy.Subscriber("/semantic/status", String, self.handle_semantic, queue_size=1)
        rospy.Subscriber("/slam_path", Path, self.handle_path, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.handle_map, queue_size=1)
        rospy.Subscriber("/clock", Clock, self.handle_clock, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.report)

    def handle_semantic(self, message: String) -> None:
        try:
            self.semantic = json.loads(message.data)
        except json.JSONDecodeError:
            self.semantic = {"error": "状态JSON无效"}

    def handle_path(self, message: Path) -> None:
        self.path_poses = len(message.poses)

    def handle_map(self, message: OccupancyGrid) -> None:
        width = message.info.width * message.info.resolution
        height = message.info.height * message.info.resolution
        self.map_size = f"{message.info.width}x{message.info.height}格 ({width:.1f}x{height:.1f}m)"

    def handle_clock(self, message: Clock) -> None:
        current_sim = message.clock.to_sec()
        current_wall = time.monotonic()
        if self.sim_time is not None and current_wall > self.wall_time:
            instant = min(2.0, max(0.0, (current_sim - self.sim_time) / (current_wall - self.wall_time)))
            self.real_time_factor = instant if self.real_time_factor == 0.0 else 0.8 * self.real_time_factor + 0.2 * instant
        self.sim_time = current_sim
        self.wall_time = current_wall

    def report(self, _event: rospy.timer.TimerEvent) -> None:
        detections = self.semantic.get("detections", [])
        names = ",".join(detections) if detections else "无"
        print(
            "[复赛实时状态] "
            f"RTF={self.real_time_factor:.2f} | SLAM轨迹={self.path_poses}位姿 | 地图={self.map_size} | "
            f"语义帧={self.semantic.get('frame', 0)} 推理={self.semantic.get('inference_ms', 0):.1f}ms "
            f"总耗时={self.semantic.get('pipeline_ms', 0):.1f}ms | 类别=[{names}] "
            f"关联点={self.semantic.get('associated_points', 0)} "
            f"几何体素={self.semantic.get('geometry_voxels', 0)} "
            f"语义体素={self.semantic.get('semantic_voxels', 0)}",
            flush=True,
        )


if __name__ == "__main__":
    rospy.init_node("openloong_runtime_monitor")
    RuntimeMonitor()
    rospy.spin()
