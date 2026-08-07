"""
rover_kinematics.launch.py
Standalone launcher for the rover_kinematics node.

This replaces the legacy qrk.yaml which used a format not supported by
`ros2 launch`. The node runs as its own OS process with a 4-thread
MultiThreadedExecutor.

Usage:
  ros2 launch rover_kinematics rover_kinematics.launch.py
  ros2 launch rover_kinematics rover_kinematics.launch.py use_sim_time:=false
  ros2 launch rover_kinematics rover_kinematics.launch.py \\
      config_file:=/path/to/custom_config.yaml

Live parameter tuning (no restart):
  ros2 param set /rover_kinematics twist_ema_alpha 0.6
  ros2 param set /rover_kinematics publish_rate 20.0
  ros2 param list /rover_kinematics

For zero-copy intra-process use with nav2, prefer:
  ros2 launch rover_kinematics rover_kinematics_component.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = FindPackageShare("rover_kinematics")

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [pkg_share, "config", "rover_config.yaml"]
        ),
        description="Absolute path to the rover_kinematics YAML config file.",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description=(
            "Use Gazebo/simulation time (true) or wall clock (false). "
            "When true, this->get_clock()->now() returns Gazebo sim time "
            "automatically — no custom /clock subscriber required."
        ),
    )

    kinematics_node = Node(
        package="rover_kinematics",
        executable="rover_kinematics_node",
        name="rover_kinematics",
        parameters=[
            LaunchConfiguration("config_file"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        kinematics_node,
    ])
