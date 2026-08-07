"""
rover_kinematics_component.launch.py
Composable-node launcher for rover_kinematics.

Loads KinematicsNode into a component_container_mt (multi-threaded) so it
shares an OS process with nav2 nodes or other components, enabling zero-copy
intra-process communication for odometry and wheel command messages.

Usage:
  ros2 launch rover_kinematics rover_kinematics_component.launch.py

  # Load additional components into the same container at runtime:
  ros2 component load /kinematics_container rover_kinematics KinematicsNode

Inspect active components:
  ros2 component list

Live parameter tuning (no restart):
  ros2 param set /rover_kinematics twist_ema_alpha 0.6
  ros2 param list /rover_kinematics
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
        description="Use Gazebo/simulation time (true) or wall clock (false).",
    )

    # component_container_mt provides a multi-threaded executor for all
    # components loaded into it.  Use component_container for single-threaded.
    container = ComposableNodeContainer(
        name="kinematics_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="rover_kinematics",
                plugin="KinematicsNode",
                name="rover_kinematics",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"use_sim_time": LaunchConfiguration("use_sim_time")},
                ],
                # Enable intra-process communication: subscribers in the same
                # container share message memory without serialisation.
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        container,
    ])
