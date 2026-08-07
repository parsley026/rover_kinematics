#include "rover_kinematics/ros/kinematics_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

/**
 * @brief Standalone entry point for rover_kinematics_node.
 *
 * Uses a 4-thread MultiThreadedExecutor so that the odometry timer, VESC
 * feedback callbacks, and service callbacks can run concurrently.
 *
 * Alternatively, KinematicsNode can be loaded into a container process for
 * zero-copy intra-process communication:
 *   ros2 launch rover_kinematics rover_kinematics_component.launch.py
 */
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  // Config path is resolved from ament_index inside the constructor.
  auto node = std::make_shared<KinematicsNode>(rclcpp::NodeOptions());

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(), 4 /* threads */);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}