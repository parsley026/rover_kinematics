cmake_minimum_required(VERSION 3.16)
project(rover_kinematics)

# ── C++ standard ─────────────────────────────────────────────────────────────
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ──────────────────────────────────────────────────────────────
find_package(ament_cmake       REQUIRED)
find_package(rclcpp            REQUIRED)
find_package(rclcpp_components REQUIRED)   # composable-node support
find_package(ament_index_cpp   REQUIRED)
find_package(geometry_msgs     REQUIRED)
find_package(nav_msgs          REQUIRED)
find_package(rcl_interfaces    REQUIRED)   # SetParametersResult for param server
find_package(realtime_tools    REQUIRED)
find_package(sensor_msgs       REQUIRED)
find_package(std_msgs          REQUIRED)
find_package(tf2               REQUIRED)
find_package(tf2_ros           REQUIRED)
find_package(Eigen3            REQUIRED)
find_package(yaml-cpp          REQUIRED)
find_package(rex_interfaces    REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs          REQUIRED)
find_package(Ceres             REQUIRED)

# ── Shared component library ──────────────────────────────────────────────────
# All node logic lives in this shared library so it can be:
#   a) linked into the standalone rover_kinematics_node executable, OR
#   b) loaded at runtime by a component_container / component_container_mt.
add_library(${PROJECT_NAME}_component SHARED
  src/ros/kinematics_node.cpp
  src/ros/kinematics_parameter_manager.cpp
  src/core/kinematics_solver.cpp
  src/core/kinematics_estimator.cpp
  src/ros/hardware_interface.cpp
)

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  ament_index_cpp
  geometry_msgs
  nav_msgs
  rcl_interfaces
  realtime_tools
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  Eigen3
  yaml-cpp
  rex_interfaces
  tf2_geometry_msgs
  std_srvs
  Ceres
)

target_link_libraries(${PROJECT_NAME}_component
  yaml-cpp
  ${CERES_LIBRARIES}
)

# Register KinematicsNode as an rclcpp_components plugin.
# This embeds plugin metadata so the node can be loaded via:
#   ros2 launch rover_kinematics rover_kinematics_component.launch.py
# or dynamically at runtime:
#   ros2 component load /ComponentManager rover_kinematics KinematicsNode
rclcpp_components_register_node(
  ${PROJECT_NAME}_component
  PLUGIN  "KinematicsNode"
  EXECUTABLE ${PROJECT_NAME}_component_node   # lightweight single-thread standalone
)

# ── Standalone executable (MultiThreadedExecutor, 4 threads) ─────────────────
# Preferred for dedicated-process deployments where full CPU utilisation matters.
add_executable(${PROJECT_NAME}_node src/main.cpp)

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_component
)

ament_target_dependencies(${PROJECT_NAME}_node rclcpp ament_index_cpp)

# ── Install ───────────────────────────────────────────────────────────────────
install(TARGETS
  ${PROJECT_NAME}_component
  ${PROJECT_NAME}_component_node
  ${PROJECT_NAME}_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

# ── Testing ───────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
