#!/usr/bin/env python3
"""Drive a repeatable curved route so SLAM and semantics remain visible during recording."""

import math

import rospy
from geometry_msgs.msg import TwistStamped


class DemoDriver:
    def __init__(self) -> None:
        self.linear_speed = float(rospy.get_param("~linear_speed", 0.35))
        self.angular_speed = float(rospy.get_param("~angular_speed", 0.12))
        self.start_delay = float(rospy.get_param("~start_delay", 5.0))
        self.publisher = rospy.Publisher("/cmd_vel", TwistStamped, queue_size=2)
        self.started_at = None
        self.circle_period = (
            2.0 * math.pi / abs(self.angular_speed)
            if abs(self.angular_speed) > 1e-6
            else math.inf
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.publish_command)
        rospy.on_shutdown(self.stop)

    def publish_command(self, _event: rospy.timer.TimerEvent) -> None:
        now = rospy.Time.now()
        if self.started_at is None and now.to_sec() > 0.0:
            self.started_at = now
        elapsed = 0.0 if self.started_at is None else (now - self.started_at).to_sec()

        command = TwistStamped()
        command.header.stamp = now
        if elapsed >= self.start_delay:
            # Each phase is one complete circle. Alternating direction at the
            # same pose makes a bounded figure eight instead of drifting away.
            phase = int((elapsed - self.start_delay) // self.circle_period) % 2
            command.twist.linear.x = self.linear_speed
            command.twist.angular.z = self.angular_speed if phase == 0 else -self.angular_speed
        self.publisher.publish(command)

        if int(elapsed * 20.0) % 200 == 0:
            rospy.loginfo_throttle(
                10.0,
                "[自动路线] 仿真时间=%.1fs 线速度=%.2fm/s 角速度=%.2frad/s",
                elapsed,
                command.twist.linear.x,
                command.twist.angular.z,
            )

    def stop(self) -> None:
        command = TwistStamped()
        command.header.stamp = rospy.Time.now()
        self.publisher.publish(command)


if __name__ == "__main__":
    rospy.init_node("demo_driver")
    DemoDriver()
    rospy.spin()
