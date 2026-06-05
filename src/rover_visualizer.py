#!/usr/bin/env python3
"""2D top-down visualizer for ackermann_rover — no Gazebo required.

Subscribes to /odom, /scan, /cmd_vel and renders:
  - Rover body + 4 wheels with Ackermann steering
  - LiDAR scan points
  - Odometry trajectory trail
  - Command velocity arrow

Usage:
  ros2 run ackermann_rover rover_visualizer
"""
import math
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from collections import deque
import threading


def _load_segments(path):
    """Load obstacle segments from .segments file (x1 y1 x2 y2 per line, # comments)."""
    if not path:
        return []
    segs = []
    try:
        with open(path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split()
                if len(parts) >= 4:
                    segs.append(((float(parts[0]), float(parts[1])),
                                 (float(parts[2]), float(parts[3]))))
        print(f"  [visualizer] loaded {len(segs)} segments from {path}")
        return segs
    except Exception as e:
        print(f"  [visualizer] failed to load map {path}: {e}")
        return []


class RoverVisualizer(Node):
    def __init__(self):
        super().__init__('rover_visualizer')

        self.declare_parameter('wheelbase', 1.2)
        self.declare_parameter('track_width', 0.8)
        self.declare_parameter('chassis_length', 1.6)
        self.declare_parameter('chassis_width', 0.9)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('max_steering_angle', 0.52)
        self.declare_parameter('lidar_offset_x', 0.65)
        self.declare_parameter('lidar_offset_y', 0.0)
        self.declare_parameter('map_file', '')

        self.wheelbase = self.get_parameter('wheelbase').value
        self.track = self.get_parameter('track_width').value
        self.chassis_l = self.get_parameter('chassis_length').value
        self.chassis_w = self.get_parameter('chassis_width').value
        self.wheel_r = self.get_parameter('wheel_radius').value
        self.max_steer = self.get_parameter('max_steering_angle').value
        self.lidar_off_x = self.get_parameter('lidar_offset_x').value
        self.lidar_off_y = self.get_parameter('lidar_offset_y').value
        map_file = self.get_parameter('map_file').value
        self.obstacles = _load_segments(map_file) if map_file else []

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.steering_angle = 0.0
        self.last_cmd_vel = None
        self.scan_ranges = []
        self.scan_angle_min = 0.0
        self.scan_angle_max = 0.0
        self.scan_angle_inc = 0.0
        self.trajectory = deque(maxlen=500)
        self._odom_msg_count = 0
        self._last_log_time = self.get_clock().now()
        self.lock = threading.Lock()

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.traj_pub = self.create_publisher(Path, '/rover/trajectory', 10)

        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('Ackermann Rover - 2D Visualizer')
        self.setup_plot()

        self.timer = self.create_timer(0.05, self.render)
        self.get_logger().info('RoverVisualizer started')

    def setup_plot(self):
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3, linestyle='--')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Ackermann Rover — 2D Top-Down View', fontsize=12, fontweight='bold')

    def odom_callback(self, msg: Odometry):
        with self.lock:
            self.odom_x = msg.pose.pose.position.x
            self.odom_y = msg.pose.pose.position.y

            q = msg.pose.pose.orientation
            siny = 2.0 * (q.w * q.z + q.x * q.y)
            cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self.odom_theta = math.atan2(siny, cosy)

            self._odom_msg_count += 1
            self.trajectory.append((self.odom_x, self.odom_y))

            path = Path()
            path.header.frame_id = 'odom'
            path.header.stamp = self.get_clock().now().to_msg()
            for (tx, ty) in list(self.trajectory)[-100:]:
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = tx
                pose.pose.position.y = ty
                pose.pose.position.z = 0.0
                path.poses.append(pose)
            self.traj_pub.publish(path)

    def scan_callback(self, msg: LaserScan):
        with self.lock:
            self.scan_angle_min = msg.angle_min
            self.scan_angle_max = msg.angle_max
            self.scan_angle_inc = msg.angle_increment
            self.scan_ranges = list(msg.ranges)

    def cmd_callback(self, msg: Twist):
        with self.lock:
            self.last_cmd_vel = msg

    def render(self):
        with self.lock:
            if not plt.fignum_exists(self.fig.number):
                return

            self.ax.clear()
            self.setup_plot()

            odom_x, odom_y, odom_theta = self.odom_x, self.odom_y, self.odom_theta

            now = self.get_clock().now()
            if (now - self._last_log_time).nanoseconds / 1e9 > 0.5:
                self.get_logger().info(
                    f"[visualizer] odom#{self._odom_msg_count} "
                    f"pos=({odom_x:.3f}, {odom_y:.3f}) theta={odom_theta:.3f} "
                    f"traj={len(self.trajectory)} scan={len(self.scan_ranges)}")
                self._last_log_time = now

            self._draw_trajectory()
            self._draw_obstacles()
            self._draw_lidar_scans(odom_x, odom_y, odom_theta)
            self._draw_rover(odom_x, odom_y, odom_theta)
            self._draw_cmd_arrow(odom_x, odom_y, odom_theta)

            view_range = 8.0
            self.ax.set_xlim(odom_x - view_range, odom_x + view_range)
            self.ax.set_ylim(odom_y - view_range, odom_y + view_range)

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def _draw_trajectory(self):
        traj = list(self.trajectory)
        if len(traj) < 2:
            return
        xs = [p[0] for p in traj]
        ys = [p[1] for p in traj]
        self.ax.plot(xs, ys, color='cyan', linewidth=1.0, alpha=0.5)

    def _draw_obstacles(self):
        for (x1, y1), (x2, y2) in self.obstacles:
            self.ax.plot([x1, x2], [y1, y2], color='#6B4226', linewidth=3, alpha=0.7, zorder=5)

    def _draw_lidar_scans(self, cx, cy, theta):
        if not self.scan_ranges:
            return
        lx = cx + self.lidar_off_x * math.cos(theta) - self.lidar_off_y * math.sin(theta)
        ly = cy + self.lidar_off_x * math.sin(theta) + self.lidar_off_y * math.cos(theta)
        xs, ys = [], []
        for i, r in enumerate(self.scan_ranges):
            if r <= 0.01 or r > 100:
                continue
            angle = self.scan_angle_min + i * self.scan_angle_inc + theta
            xs.append(lx + r * math.cos(angle))
            ys.append(ly + r * math.sin(angle))
        if xs:
            self.ax.scatter(xs, ys, s=1, c='lime', alpha=0.4, marker='.')

    def _draw_rover(self, cx, cy, theta):
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)

        def transform(x, y):
            return (
                cx + x * cos_t - y * sin_t,
                cy + x * sin_t + y * cos_t
            )

        half_l = self.chassis_l / 2
        half_w = self.chassis_w / 2
        corners = [
            transform(-half_l, -half_w),
            transform(-half_l, half_w),
            transform(half_l, half_w),
            transform(half_l, -half_w),
        ]
        poly = plt.Polygon(corners, closed=True, facecolor='#264D8C',
                           edgecolor='#1A3366', linewidth=2, alpha=0.85,
                           zorder=10)
        self.ax.add_patch(poly)

        arrow_len = 0.4
        self.ax.arrow(cx, cy,
                      arrow_len * cos_t, arrow_len * sin_t,
                      head_width=0.25, head_length=0.2, fc='orange', ec='orange',
                      linewidth=2, zorder=15)

        refs = [
            transform(-half_l * 0.8, half_w * 0.85),
            transform(-half_l * 0.8, -half_w * 0.85),
            transform(half_l * 0.8, half_w * 0.85),
            transform(half_l * 0.8, -half_w * 0.85),
        ]
        for pos in refs:
            self.ax.plot(pos[0], pos[1], marker='o', color='red',
                         markersize=5, zorder=12)

        if self.last_cmd_vel:
            linear = self.last_cmd_vel.linear.x
            angular = self.last_cmd_vel.angular.z
            if abs(linear) > 0.001:
                sign = 1.0 if linear > 0 else -1.0
                steer = sign * math.atan2(self.wheelbase * angular, abs(linear))
            else:
                steer = 0.0
            steer = max(-self.max_steer, min(self.max_steer, steer))
        else:
            steer = 0.0

        self.steering_angle = steer

        front_x = self.wheelbase / 2.0
        front_track = self.track / 2.0
        ww = 0.06

        inner_steer, outer_steer = steer, steer
        if abs(steer) > 1e-6:
            R = self.wheelbase / math.tan(abs(steer))
            inner_angle = math.atan(self.wheelbase / (R - self.track / 2.0))
            outer_angle = math.atan(self.wheelbase / (R + self.track / 2.0))
            if steer > 0:
                inner_steer, outer_steer = inner_angle, outer_angle
            else:
                inner_steer, outer_steer = -outer_angle, -inner_angle

        wheels = [
            (-front_x, front_track, 0.0),
            (-front_x, -front_track, 0.0),
            (front_x, front_track, inner_steer),
            (front_x, -front_track, outer_steer),
        ]

        for wx, wy, wsteer in wheels:
            dx = 0.0
            dy = ww / 2
            cos_s = math.cos(theta + wsteer)
            sin_s = math.sin(theta + wsteer)
            cos_p = math.cos(theta + wsteer + math.pi / 2)
            sin_p = math.sin(theta + wsteer + math.pi / 2)

            pts = [
                transform(wx + cos_s * (-self.wheel_r) + cos_p * (-dy),
                          wy + sin_s * (-self.wheel_r) + sin_p * (-dy)),
                transform(wx + cos_s * (-self.wheel_r) + cos_p * dy,
                          wy + sin_s * (-self.wheel_r) + sin_p * dy),
                transform(wx + cos_s * self.wheel_r + cos_p * dy,
                          wy + sin_s * self.wheel_r + sin_p * dy),
                transform(wx + cos_s * self.wheel_r + cos_p * (-dy),
                          wy + sin_s * self.wheel_r + sin_p * (-dy)),
            ]
            is_front = abs(wx - front_x) < 0.01
            color = '#222222' if is_front else '#444444'
            wheel_poly = plt.Polygon(pts, closed=True, facecolor=color,
                                      edgecolor='#111111', linewidth=1,
                                      zorder=11)
            self.ax.add_patch(wheel_poly)

    def _draw_cmd_arrow(self, cx, cy, theta):
        if self.last_cmd_vel is None:
            return
        linear = self.last_cmd_vel.linear.x
        angular = self.last_cmd_vel.angular.z

        if abs(linear) < 0.01:
            return

        if abs(angular) > 0.001:
            R = linear / angular
            turn_x = cx - R * math.sin(theta)
            turn_y = cy + R * math.cos(theta)
            self.ax.plot(turn_x, turn_y, 'm+', markersize=10, zorder=8)


def main():
    rclpy.init()
    node = RoverVisualizer()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            plt.pause(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')


if __name__ == '__main__':
    main()
