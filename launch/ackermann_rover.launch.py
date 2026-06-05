"""ackermann_rover launch — Bring up all rover nodes + visualization

Jetson Nano rover: VESC motors + servo steering + LiDAR + IMU + wheel encoders

All vehicle parameters come from config/rover_params.yaml — change one value there
and every node, the visualizer, and the URDF model all update together.

Usage:
  ros2 launch ackermann_rover ackermann_rover.launch.py
  ros2 launch ackermann_rover ackermann_rover.launch.py simulated:=false
  ros2 launch ackermann_rover ackermann_rover.launch.py rviz:=true
  ros2 launch ackermann_rover ackermann_rover.launch.py visualizer:=true
  ros2 launch ackermann_rover ackermann_rover.launch.py map_file:=/path/to/my_map.segments
"""
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import LifecycleNode, LifecycleTransition, Node
import lifecycle_msgs.msg


_TRANSITION_CONFIGURE = lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
_TRANSITION_ACTIVATE = lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE


def _load_params(share_dir):
    """Load rover_params.yaml and return (shared_dict, node_dicts)."""
    path = os.path.join(share_dir, "config", "rover_params.yaml")
    with open(path, "r") as f:
        raw = yaml.safe_load(f)
    shared = raw.get("/**", {}).get("ros__parameters", {})
    nodes = {}
    for key, value in raw.items():
        if key.startswith("/") and key != "/**":
            nodes[key.lstrip("/")] = value.get("ros__parameters", {})
    return shared, nodes


def _xacro_command(share_dir, params):
    """Build a Command that runs xacro with geometric params matching the YAML."""
    xacro_path = os.path.join(share_dir, "urdf", "rover.urdf.xacro")
    xacro_args = (
        f"wheelbase:={params.get('wheelbase', 1.2)} "
        f"track_width:={params.get('track_width', 0.8)} "
        f"wheel_radius:={params.get('wheel_radius', 0.15)} "
        f"chassis_length:={params.get('chassis_length', 1.6)} "
        f"chassis_width:={params.get('chassis_width', 0.9)}"
    )
    return Command(["xacro ", xacro_args, " ", xacro_path])


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
    pkg_name = "ackermann_rover"
    share = get_package_share_directory(pkg_name)
    shared, node_params = _load_params(share)

    simulated = LaunchConfiguration("simulated", default="true")
    use_rviz = LaunchConfiguration("rviz", default="false")
    use_visualizer = LaunchConfiguration("visualizer", default="true")
    default_map = os.path.join(share, "maps", "gym.segments")
    map_file = LaunchConfiguration("map_file", default=default_map)

    sim_arg = DeclareLaunchArgument(
        "simulated", default_value="true",
        description="Run in simulated mode (no hardware I/O)")

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="false",
        description="Launch RViz2 for 3D robot visualization")

    viz_arg = DeclareLaunchArgument(
        "visualizer", default_value="true",
        description="Launch 2D matplotlib visualizer (lightweight, no GPU needed)")

    map_arg = DeclareLaunchArgument(
        "map_file", default_value=default_map,
        description="Path to .segments map file")

    # ── Robot State Publisher (xacro → URDF → TF) ─────────────
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": _xacro_command(share, shared),
            "use_sim_time": False,
        }],
    )

    # ── Joint State Bootstrap ─────────────────────────────────
    joint_state_bootstrap = Node(
        package=pkg_name,
        executable="joint_state_bootstrap.py",
        name="joint_state_bootstrap",
        output="screen",
    )

    # Helper: merge shared params + node-specific params + extras
    def _make_params(node_name, **extras):
        result = [shared, node_params.get(node_name, {})]
        if extras:
            result.append(extras)
        return result

    # ── Rover Controller ─────────────────────────────────────
    rover_controller = LifecycleNode(
        package=pkg_name, executable="rover_controller_node",
        name="rover_controller", namespace="", output="screen",
        parameters=_make_params("rover_controller", simulated=simulated),
    )

    # ── VESC Motor Driver ────────────────────────────────────
    vesc_driver = LifecycleNode(
        package=pkg_name, executable="vesc_driver_node",
        name="vesc_driver", namespace="", output="screen",
        parameters=_make_params("vesc_driver", simulated=simulated),
    )

    # ── Steering Servo ───────────────────────────────────────
    steering_servo = LifecycleNode(
        package=pkg_name, executable="steering_servo_node",
        name="steering_servo", namespace="", output="screen",
        parameters=_make_params("steering_servo", simulated=simulated),
    )

    # ── Encoder Odometry ─────────────────────────────────────
    encoder_odometry = LifecycleNode(
        package=pkg_name, executable="encoder_odometry_node",
        name="encoder_odometry", namespace="", output="screen",
        parameters=_make_params("encoder_odometry", simulated=simulated),
    )

    # ── LiDAR Driver ─────────────────────────────────────────
    lidar_driver = LifecycleNode(
        package=pkg_name, executable="lidar_driver_node",
        name="lidar_driver", namespace="", output="screen",
        parameters=_make_params("lidar_driver",
                                simulated=simulated, map_file=map_file),
    )

    # ── IMU Driver ───────────────────────────────────────────
    imu_driver = LifecycleNode(
        package=pkg_name, executable="imu_driver_node",
        name="imu_driver", namespace="", output="screen",
        parameters=_make_params("imu_driver", simulated=simulated),
    )

    # ── RViz2 (optional) ─────────────────────────────────────
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(share, "config", "rover.rviz")],
        output="screen",
    )

    # ── 2D Visualizer ────────────────────────────────────────
    visualizer_node = Node(
        package=pkg_name, executable="rover_visualizer.py",
        name="rover_visualizer",
        condition=IfCondition(use_visualizer),
        output="screen",
        parameters=_make_params("rover_visualizer", map_file=map_file),
    )

    # ── Lifecycle transitions ────────────────────────────────
    lifecycle_node_names = [
        "rover_controller", "vesc_driver", "steering_servo",
        "encoder_odometry", "lidar_driver", "imu_driver",
    ]
    configure_timer, activate_timer = _make_lifecycle_transitions(
        lifecycle_node_names)

    return LaunchDescription([
        sim_arg, rviz_arg, viz_arg, map_arg,
        robot_state_pub, joint_state_bootstrap,
        rover_controller, vesc_driver, steering_servo,
        encoder_odometry, lidar_driver, imu_driver,
        rviz_node, visualizer_node,
        configure_timer, activate_timer,
    ])
