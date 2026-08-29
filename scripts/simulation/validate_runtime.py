#!/usr/bin/env python3
"""Observe the live semifinal pipeline and enforce its recording-time gates."""

import argparse
import json
import math
import statistics
import time
from pathlib import Path

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Odometry, Path as RosPath
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class RuntimeValidator:
    def __init__(self) -> None:
        self.samples = {}
        self.first_position = None
        self.last_position = None
        self.previous_position = None
        self.distance_traveled = 0.0
        self.latest_map = None
        self.latest_path_poses = 0
        self.semantic_statuses = []
        self.clock_first = None
        self.clock_last = None
        self.clock_wall_first = None
        self.clock_wall_last = None
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        topics = {
            "camera": ("/camera/image", Image),
            "lidar": ("/velodyne_points", PointCloud2),
            "scan": ("/scan", LaserScan),
            "odometry": ("/state_estimation", Odometry),
            "slam_map": ("/map", OccupancyGrid),
            "slam_path": ("/slam_path", RosPath),
            "semantic_image": ("/semantic/image", Image),
            "semantic_cloud": ("/semantic/cloud", PointCloud2),
            "geometry_map": ("/semantic/geometry_map", PointCloud2),
            "semantic_map": ("/semantic/map", PointCloud2),
            "semantic_markers": ("/semantic/markers", MarkerArray),
            "semantic_status": ("/semantic/status", String),
        }
        self.subscribers = [
            rospy.Subscriber(topic, message_type, self.observe, callback_args=name, queue_size=10)
            for name, (topic, message_type) in topics.items()
        ]
        rospy.Subscriber("/state_estimation", Odometry, self.observe_odometry, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.observe_map, queue_size=2)
        rospy.Subscriber("/slam_path", RosPath, self.observe_path, queue_size=2)
        rospy.Subscriber("/semantic/status", String, self.observe_semantic, queue_size=10)
        rospy.Subscriber("/clock", Clock, self.observe_clock, queue_size=100)

    @staticmethod
    def message_stamp(message) -> float:
        header = getattr(message, "header", None)
        if header is not None and not header.stamp.is_zero():
            return header.stamp.to_sec()
        return rospy.Time.now().to_sec()

    def observe(self, message, name: str) -> None:
        stamp = self.message_stamp(message)
        sample = self.samples.setdefault(name, {"count": 0, "first": stamp, "last": stamp})
        sample["count"] += 1
        sample["last"] = stamp

    def observe_odometry(self, message: Odometry) -> None:
        position = message.pose.pose.position
        xyz = (position.x, position.y, position.z)
        if self.first_position is None:
            self.first_position = xyz
        if self.previous_position is not None:
            step = math.dist(self.previous_position, xyz)
            if step < 0.05:
                self.distance_traveled += step
        self.previous_position = xyz
        self.last_position = xyz

    def observe_map(self, message: OccupancyGrid) -> None:
        self.latest_map = {
            "width": message.info.width,
            "height": message.info.height,
            "resolution": message.info.resolution,
        }

    def observe_path(self, message: RosPath) -> None:
        self.latest_path_poses = len(message.poses)

    def observe_semantic(self, message: String) -> None:
        try:
            self.semantic_statuses.append(json.loads(message.data))
        except json.JSONDecodeError:
            pass

    def observe_clock(self, message: Clock) -> None:
        now = time.monotonic()
        stamp = message.clock.to_sec()
        if self.clock_first is None:
            self.clock_first = stamp
            self.clock_wall_first = now
        self.clock_last = stamp
        self.clock_wall_last = now

    def frame_exists(self, target: str, source: str) -> bool:
        try:
            self.tf_buffer.lookup_transform(target, source, rospy.Time(0), rospy.Duration(0.5))
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def report(self, wall_duration: float) -> tuple[dict, list[str]]:
        rates = {}
        for name, sample in self.samples.items():
            elapsed = sample["last"] - sample["first"]
            rates[name] = (sample["count"] - 1) / elapsed if elapsed > 0.0 else 0.0

        movement = 0.0
        if self.first_position is not None and self.last_position is not None:
            movement = math.dist(self.first_position, self.last_position)
        rtf = 0.0
        if self.clock_first is not None and self.clock_wall_last > self.clock_wall_first:
            rtf = (self.clock_last - self.clock_first) / (self.clock_wall_last - self.clock_wall_first)

        inference = [float(item.get("inference_ms", 0.0)) for item in self.semantic_statuses]
        pipeline = [float(item.get("pipeline_ms", 0.0)) for item in self.semantic_statuses]
        detected = sorted({name for item in self.semantic_statuses for name in item.get("detections", [])})
        tf_checks = {
            "map_to_odom": self.frame_exists("map", "odom"),
            "odom_to_sensor": self.frame_exists("odom", "sensor"),
            "sensor_to_velodyne": self.frame_exists("sensor", "velodyne"),
            "sensor_to_camera": self.frame_exists("sensor", "camera"),
        }
        report = {
            "wall_duration_s": round(wall_duration, 2),
            "real_time_factor": round(rtf, 3),
            "rates_hz": {name: round(rate, 3) for name, rate in sorted(rates.items())},
            "message_counts": {
                name: sample["count"] for name, sample in sorted(self.samples.items())
            },
            "vehicle_displacement_m": round(movement, 3),
            "vehicle_distance_traveled_m": round(self.distance_traveled, 3),
            "slam_path_poses": self.latest_path_poses,
            "slam_map": self.latest_map,
            "semantic_frames": len(self.semantic_statuses),
            "semantic_classes_seen": detected,
            "inference_ms_median": round(statistics.median(inference), 2) if inference else None,
            "inference_ms_p95": round(sorted(inference)[int(0.95 * (len(inference) - 1))], 2) if inference else None,
            "pipeline_ms_p95": round(sorted(pipeline)[int(0.95 * (len(pipeline) - 1))], 2) if pipeline else None,
            "tf": tf_checks,
        }

        failures = []
        minimum_rates = {
            "camera": 12.0,
            "lidar": 4.0,
            "scan": 4.0,
            "odometry": 150.0,
            "slam_map": 0.5,
            "semantic_image": 4.0,
            "semantic_cloud": 4.0,
            "geometry_map": 0.4,
            "semantic_map": 0.4,
            "semantic_markers": 4.0,
            "semantic_status": 4.0,
        }
        for name, minimum in minimum_rates.items():
            if rates.get(name, 0.0) < minimum:
                failures.append(f"{name} rate {rates.get(name, 0.0):.2f} Hz < {minimum:.2f} Hz")
        if rtf < 0.8:
            failures.append(f"real-time factor {rtf:.2f} < 0.80")
        if self.distance_traveled < 10.0:
            failures.append(f"vehicle distance {self.distance_traveled:.2f} m < 10.00 m")
        if self.latest_path_poses < 20:
            failures.append(f"SLAM path only has {self.latest_path_poses} poses")
        if not detected:
            failures.append("no model-provided semantic class was detected")
        if inference and report["inference_ms_p95"] > 160.0:
            failures.append(f"semantic inference p95 {report['inference_ms_p95']:.1f} ms > 160 ms")
        if pipeline and report["pipeline_ms_p95"] > 400.0:
            failures.append(f"semantic pipeline p95 {report['pipeline_ms_p95']:.1f} ms > 400 ms")
        for name, connected in tf_checks.items():
            if not connected:
                failures.append(f"TF check failed: {name}")
        report["passed"] = not failures
        report["failures"] = failures
        return report, failures


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    rospy.init_node("openloong_runtime_validator", anonymous=True)
    validator = RuntimeValidator()
    started = time.monotonic()
    deadline = started + args.duration
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        time.sleep(0.1)
    wall_duration = time.monotonic() - started
    report, failures = validator.report(wall_duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if failures:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
