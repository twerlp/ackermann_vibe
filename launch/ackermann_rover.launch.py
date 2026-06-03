"""ackermann_rover launch — Bring up all rover nodes + visualization

Jetson Nano rover: VESC motors + servo steering + LiDAR + IMU + wheel encoders

Usage:
  ros2 launch ackermann_rover ackermann_rover.launch.py
  ros2 launch ackermann_rover ackermann_rover.launch.py simulated:=false
  ros2 launch ackermann_rover ackermann_rover.launch.py rviz:=true
  ros2 launch ackermann_rover ackermann_rover.launch.py visualizer:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node
import lifecycle_msgs.msg


_TRANSITION_CONFIGURE = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
_TRANSITION_ACTIVATE = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE


def _make_lifecycle_transitions(node_names):
    """Return (configure_timer, activate_timer) that stagger-transition all nodes."""
    configure_action = LifecycleTransition(
        lifecycle_node_names=node_names,
        transition_ids=[_TRANSITION_CONFIGURE],
    )
    activate_action = LifecycleTransition(
        lifecycle_node_names=node_names,
        transition_ids=[_TRANSITION_ACTIVATE],
    )
    configure_timer = TimerAction(period=2.0, actions=[configure_action])
    activate_timer = TimerAction(period=4.5, actions=[activate_action])
    return configure_timer, activate_timer


def generate_launch_description() -> LaunchDescription:
    simulated = LaunchConfiguration("simulated", default="true")
    use_rviz = LaunchConfiguration("rviz", default="false")
    use_visualizer = LaunchConfiguration("visualizer", default="true")

    sim_arg = DeclareLaunchArgument(
        "simulated", default_value="true",
        description="Run in simulated mode (no hardware I/O)")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Launch RViz2 for 3D robot visualization")

    viz_arg = DeclareLaunchArgument(
        "visualizer", default_value="true",
        description="Launch 2D matplotlib visualizer (lightweight, no GPU needed)")

    pkg_name = "ackermann_rover"
    share = get_package_share_directory(pkg_name)

    # ── Robot State Publisher (xacro → URDF → TF) ─────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command([
                "xacro ", os.path.join(share, "urdf", "rover.urdf.xacro")]),
            "use_sim_time": False,
        }],
    )
    # ── Joint State Bootstrap (zero states at startup, rover_controller overrides) ──
    joint_state_bootstrap = Node(
        package=pkg_name,
        executable="joint_state_bootstrap.py",
        name="joint_state_bootstrap",
        output="screen",
    )

    # ── Rover Controller ─────────────────────────────────────
    rover_controller = LifecycleNode(
        package=pkg_name,
        executable="rover_controller_node",
        name="rover_controller",
        namespace="",
        output="screen",
        parameters=[{"simulated": simulated}],
    )

    # ── VESC Motor Driver ────────────────────────────────────
    vesc_driver = LifecycleNode(
        package=pkg_name,
        executable="vesc_driver_node",
        name="vesc_driver",
        namespace="",
        output="screen",
        parameters=[{
            "simulated": simulated,
            "serial_port": "/dev/vesc",
            "baud_rate": 115200,
            "max_duty": 0.95,
            "max_current": 40.0,
        }],
    )

    # ── Steering Servo ───────────────────────────────────────
    steering_servo = LifecycleNode(
        package=pkg_name,
        executable="steering_servo_node",
        name="steering_servo",
        namespace="",
        output="screen",
        parameters=[{
            "simulated": simulated,
            "pwm_pin": 12,
            "max_angle_rad": 0.52,
            "min_angle_rad": -0.52,
            "center_pulse_us": 1500.0,
            "range_pulse_us": 500.0,
        }],
    )

    # ── Encoder Odometry ─────────────────────────────────────
    encoder_odometry = LifecycleNode(
        package=pkg_name,
        executable="encoder_odometry_node",
        name="encoder_odometry",
        namespace="",
        output="screen",
        parameters=[{
            "simulated": simulated,
            "wheel_radius": 0.15,
            "wheelbase": 1.2,
            "track_width": 0.8,
            "max_steering_angle": 0.52,
            "ticks_per_rev": 1024,
            "odom_rate": 50.0,
        }],
    )

    # ── LiDAR Driver ─────────────────────────────────────────
    lidar_driver = LifecycleNode(
        package=pkg_name,
        executable="lidar_driver_node",
        name="lidar_driver",
        namespace="",
        output="screen",
        parameters=[{
            "simulated": simulated,
            "serial_port": "/dev/lidar",
            "baud_rate": 230400,
            "scan_rate": 10.0,
            "range_min": 0.12,
            "range_max": 12.0,
            "num_samples": 360,
        }],
    )

    # ── IMU Driver ───────────────────────────────────────────
    imu_driver = LifecycleNode(
        package=pkg_name,
        executable="imu_driver_node",
        name="imu_driver",
        namespace="",
        output="screen",
        parameters=[{
            "simulated": simulated,
            "i2c_device": "/dev/i2c-1",
            "i2c_address": 0x68,
            "imu_rate": 100.0,
            "frame_id": "imu_link",
        }],
    )

    # ── RViz2 (optional) ─────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(share, "config", "rover.rviz")],
        output="screen",
    )

    # ── 2D Visualizer (lightweight, no Gazebo needed) ─────────
    visualizer_node = Node(
        package=pkg_name,
        executable="rover_visualizer.py",
        name="rover_visualizer",
        condition=IfCondition(use_visualizer),
        output="screen",
    )

    # ── Lifecycle transitions (configure then activate) ──────
    lifecycle_node_names = [
        "rover_controller", "vesc_driver", "steering_servo",
        "encoder_odometry", "lidar_driver", "imu_driver",
    ]
    configure_timer, activate_timer = _make_lifecycle_transitions(lifecycle_node_names)

    return LaunchDescription([
        sim_arg,
        rviz_arg,
        viz_arg,
        robot_state_pub,
        joint_state_bootstrap,
        rover_controller,
        vesc_driver,
        steering_servo,
        encoder_odometry,
        lidar_driver,
        imu_driver,
        rviz_node,
        visualizer_node,
        configure_timer,
        activate_timer,
    ])
