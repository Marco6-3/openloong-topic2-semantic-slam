#!/usr/bin/env python3
"""把 nav_msgs/Odometry 实时转换为 RViz 可用的动态 TF。"""

from __future__ import annotations

import argparse

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/Odometry")
    parser.add_argument("--parent-frame", default="camera_init")
    parser.add_argument("--child-frame", default="body")
    return parser.parse_args()


class OdometryTfBroadcaster(Node):
    def __init__(self, topic: str, parent_frame: str, child_frame: str) -> None:
        super().__init__("odometry_tf_broadcaster")
        self.parent_frame = parent_frame
        self.child_frame = child_frame
        self.broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(Odometry, topic, self.on_odometry, 50)

    def on_odometry(self, message: Odometry) -> None:
        transform = TransformStamped()
        transform.header = message.header
        transform.header.frame_id = message.header.frame_id or self.parent_frame
        transform.child_frame_id = message.child_frame_id or self.child_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = OdometryTfBroadcaster(args.topic, args.parent_frame, args.child_frame)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
