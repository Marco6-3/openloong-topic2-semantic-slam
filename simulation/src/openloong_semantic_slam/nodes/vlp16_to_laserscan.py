#!/usr/bin/env python3
"""Collapse several near-horizontal VLP-16 rings into a stable 2D scan."""

import math

import numpy as np
import rospy
from sensor_msgs import point_cloud2
from sensor_msgs.msg import LaserScan, PointCloud2


class Vlp16ToLaserScan:
    def __init__(self) -> None:
        self.min_height = float(rospy.get_param("~min_height", -0.35))
        self.max_height = float(rospy.get_param("~max_height", 0.35))
        self.range_min = float(rospy.get_param("~range_min", 0.30))
        self.range_max = float(rospy.get_param("~range_max", 25.0))
        self.bins = int(rospy.get_param("~bins", 720))
        self.target_frame = str(rospy.get_param("~target_frame", "velodyne"))
        self.publisher = rospy.Publisher("scan", LaserScan, queue_size=2)
        self.subscriber = rospy.Subscriber(
            "velodyne_points", PointCloud2, self.handle_cloud, queue_size=1
        )
        self.frame_count = 0

    def handle_cloud(self, cloud: PointCloud2) -> None:
        xyz = np.asarray(
            list(point_cloud2.read_points(cloud, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=np.float32,
        )
        ranges = np.full(self.bins, np.inf, dtype=np.float32)
        if xyz.size:
            radius = np.hypot(xyz[:, 0], xyz[:, 1])
            valid = (
                (xyz[:, 2] >= self.min_height)
                & (xyz[:, 2] <= self.max_height)
                & (radius >= self.range_min)
                & (radius <= self.range_max)
            )
            points = xyz[valid]
            radius = radius[valid]
            if points.size:
                angles = np.arctan2(points[:, 1], points[:, 0])
                indices = np.floor((angles + math.pi) * self.bins / (2.0 * math.pi)).astype(np.int32)
                indices = np.clip(indices, 0, self.bins - 1)
                np.minimum.at(ranges, indices, radius)

        scan = LaserScan()
        scan.header = cloud.header
        scan.header.frame_id = self.target_frame
        scan.angle_min = -math.pi
        scan.angle_max = math.pi
        scan.angle_increment = 2.0 * math.pi / self.bins
        scan.scan_time = 0.2
        scan.time_increment = 0.0
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)

        self.frame_count += 1
        if self.frame_count % 25 == 0:
            finite = int(np.count_nonzero(np.isfinite(ranges)))
            rospy.loginfo("[SLAM输入] VLP16帧=%d 有效方位=%d/%d", self.frame_count, finite, self.bins)


if __name__ == "__main__":
    rospy.init_node("vlp16_to_laserscan")
    Vlp16ToLaserScan()
    rospy.spin()
