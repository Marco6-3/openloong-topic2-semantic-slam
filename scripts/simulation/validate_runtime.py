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
from gazebo_msgs.msg import ModelStates
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
        self.ground_truth_objects = {}
        self.target_position_errors = []
        self.target_position_errors_by_class = {}
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
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.observe_ground_truth, queue_size=2)
        rospy.Subscriber("/semantic/markers", MarkerArray, self.observe_markers, queue_size=10)
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

    @staticmethod
    def ground_truth_class(model_name: str) -> str | None:
        name = model_name.lower()
        if "person" in name or "citizen" in name:
            return "person"
        if "refrigerator" in name:
            return "refrigerator"
        if "cocacola" in name:
            return "bottle"
        if "bed" in name:
            return "bed"
        if "chair" in name:
            return "chair"
        if "sofa" in name:
            return "couch"
        if "tv_01" in name:
            return "tv"
        if "vase" in name:
            return "vase"
        if "table" in name:
            return "dining table"
        return None

    def observe_ground_truth(self, message: ModelStates) -> None:
        objects = {}
        for name, pose in zip(message.name, message.pose):
            class_name = self.ground_truth_class(name)
            if class_name is not None:
                objects.setdefault(class_name, []).append(
                    (pose.position.x, pose.position.y, pose.position.z)
                )
        self.ground_truth_objects = objects

    @staticmethod
    def marker_class(text: str) -> str | None:
        for name in (
            "dining table",
            "refrigerator",
            "bottle",
            "person",
            "chair",
            "couch",
            "bed",
            "tv",
            "vase",
        ):
            if text.startswith(name + " "):
                return name
        return None

    def observe_markers(self, message: MarkerArray) -> None:
        if not self.ground_truth_objects:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "map", rospy.Time(0), rospy.Duration(0.05)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return
        q = transform.transform.rotation
        norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w
        if norm < 1e-12:
            return
        scale = 2.0 / norm
        rotation = (
            (1.0 - scale * (q.y * q.y + q.z * q.z), scale * (q.x * q.y - q.z * q.w)),
            (scale * (q.x * q.y + q.z * q.w), 1.0 - scale * (q.x * q.x + q.z * q.z)),
        )
        translation = transform.transform.translation
        for marker in message.markers:
            class_name = self.marker_class(marker.text)
            targets = self.ground_truth_objects.get(class_name or "", [])
            if not targets:
                continue
            x_map, y_map = marker.pose.position.x, marker.pose.position.y
            x_odom = rotation[0][0] * x_map + rotation[0][1] * y_map + translation.x
            y_odom = rotation[1][0] * x_map + rotation[1][1] * y_map + translation.y
            error = min(math.hypot(x_odom - x, y_odom - y) for x, y, _z in targets)
            self.target_position_errors.append(error)
            self.target_position_errors_by_class.setdefault(class_name, []).append(error)

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
        image_delta = [float(item.get("image_delta_ms", 0.0)) for item in self.semantic_statuses]
        final_semantic = self.semantic_statuses[-1] if self.semantic_statuses else {}
        stable_voxels = int(final_semantic.get("semantic_voxels", 0))
        candidate_voxels = int(final_semantic.get("semantic_candidate_voxels", stable_voxels))
        transient_voxels = int(final_semantic.get("semantic_transient_voxels", 0))
        detected = sorted({name for item in self.semantic_statuses for name in item.get("detections", [])})
        policy_rejected = sum(
            int(item.get("policy_rejected", 0)) for item in self.semantic_statuses
        )
        policy_rejected_classes = sorted(
            {
                name
                for item in self.semantic_statuses
                for name in item.get("policy_rejected_classes", [])
            }
        )
        association_totals = {
            key: sum(
                int(item.get("association", {}).get(key, 0))
                for item in self.semantic_statuses
            )
            for key in (
                "raw_mask_matches",
                "boundary_rejected",
                "depth_rejected",
                "low_confidence_rejected",
                "associated",
            )
        }
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
            "semantic_policy": {
                "rejected_candidates": policy_rejected,
                "rejected_classes": policy_rejected_classes,
            },
            "association_totals": association_totals,
            "target_position_xy_error_m": {
                "samples": len(self.target_position_errors),
                "median": round(statistics.median(self.target_position_errors), 3)
                if self.target_position_errors
                else None,
                "p95": round(
                    sorted(self.target_position_errors)[
                        int(0.95 * (len(self.target_position_errors) - 1))
                    ],
                    3,
                )
                if self.target_position_errors
                else None,
                "by_class": {
                    name: {
                        "samples": len(errors),
                        "median": round(statistics.median(errors), 3),
                        "p95": round(
                            sorted(errors)[int(0.95 * (len(errors) - 1))], 3
                        ),
                    }
                    for name, errors in sorted(self.target_position_errors_by_class.items())
                },
            },
            "inference_ms_median": round(statistics.median(inference), 2) if inference else None,
            "inference_ms_p95": round(sorted(inference)[int(0.95 * (len(inference) - 1))], 2) if inference else None,
            "pipeline_ms_p95": round(sorted(pipeline)[int(0.95 * (len(pipeline) - 1))], 2) if pipeline else None,
            "image_delta_ms_p95": round(sorted(image_delta)[int(0.95 * (len(image_delta) - 1))], 2) if image_delta else None,
            "semantic_voxels_final": {
                "stable": stable_voxels,
                "candidate": candidate_voxels,
                "transient_filtered": transient_voxels,
                "stable_ratio": round(stable_voxels / candidate_voxels, 4) if candidate_voxels else None,
            },
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
        if image_delta and report["image_delta_ms_p95"] > 120.0:
            failures.append(f"camera-lidar timestamp delta p95 {report['image_delta_ms_p95']:.1f} ms > 120 ms")
        target_error = report["target_position_xy_error_m"]
        if target_error["samples"] >= 20 and target_error["p95"] > 1.0:
            failures.append(
                f"semantic target position p95 {target_error['p95']:.2f} m > 1.00 m"
            )
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
