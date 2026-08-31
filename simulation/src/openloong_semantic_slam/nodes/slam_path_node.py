#!/usr/bin/env python3
"""Publish the estimated SLAM trajectory from the map-to-sensor TF chain."""

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


class SlamPathNode:
    def __init__(self) -> None:
        self.map_frame = str(rospy.get_param("~map_frame", "map"))
        self.base_frame = str(rospy.get_param("~base_frame", "sensor"))
        self.max_poses = int(rospy.get_param("~max_poses", 12000))
        self.buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.publisher = rospy.Publisher("slam_path", Path, queue_size=1, latch=True)
        self.path = Path()
        self.path.header.frame_id = self.map_frame
        self.last_xy = None
        self.timer = rospy.Timer(rospy.Duration(0.2), self.update)

    def update(self, _event: rospy.timer.TimerEvent) -> None:
        try:
            transform = self.buffer.lookup_transform(
                self.map_frame, self.base_frame, rospy.Time(0), rospy.Duration(0.05)
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return

        translation = transform.transform.translation
        xy = (translation.x, translation.y)
        if self.last_xy is not None:
            dx = xy[0] - self.last_xy[0]
            dy = xy[1] - self.last_xy[1]
            if dx * dx + dy * dy < 0.0025:
                return

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = translation.x
        pose.pose.position.y = translation.y
        pose.pose.position.z = translation.z
        pose.pose.orientation = transform.transform.rotation
        self.path.poses.append(pose)
        if len(self.path.poses) > self.max_poses:
            self.path.poses = self.path.poses[-self.max_poses :]
        self.path.header.stamp = transform.header.stamp
        self.publisher.publish(self.path)
        self.last_xy = xy


if __name__ == "__main__":
    rospy.init_node("slam_path_node")
    SlamPathNode()
    rospy.spin()
