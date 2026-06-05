#!/usr/bin/env python3
"""Publishes zero joint states at startup so robot_state_publisher can compute
non-fixed-joint TFs before rover_controller activates.

Subscribes to /joint_states and latches on the first message with non-zero
velocity from rover_controller, then permanently stops its own publishing.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBootstrap(Node):
    def __init__(self):
        super().__init__("joint_state_bootstrap")
        self.pub = self.create_publisher(JointState, "joint_states", 10)
        self.timer = self.create_timer(0.5, self.publish_zero)
        self._latched = False
        self._latch_sub = self.create_subscription(
            JointState, "joint_states",
            self._latch_callback, 10)
        self.joint_names = [
            "rl_wheel_joint", "rr_wheel_joint",
            "fl_wheel_joint", "fr_wheel_joint",
            "fl_steering_joint", "fr_steering_joint",
        ]
        self.publish_zero()
        self.get_logger().info("JointStateBootstrap started, waiting for rover_controller")

    def publish_zero(self):
        if self._latched:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        n = len(self.joint_names)
        msg.position = [0.0] * n
        msg.velocity = [0.0] * n
        self.pub.publish(msg)

    def _latch_callback(self, msg):
        if self._latched:
            return
        has_vel = any(abs(v) > 0.001 for v in msg.velocity) if msg.velocity else False
        has_pos = msg.velocity and any(abs(p) > 0.001 for p in msg.position)
        if has_vel or has_pos:
            self._latched = True
            self.timer.cancel()
            self.destroy_subscription(self._latch_sub)
            self.get_logger().info(
                "RoverController takeover detected — bootstrap disabled")


def main():
    rclpy.init()
    node = JointStateBootstrap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
