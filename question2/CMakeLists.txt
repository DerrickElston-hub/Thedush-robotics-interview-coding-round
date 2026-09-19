cmake_minimum_required(VERSION 3.8)

project(ros2_coding_round)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------
# Dependencies
# ---------------------------------------------------------

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)

find_package(pluginlib REQUIRED)

# ---------------------------------------------------------
# Q1 - TF2 Frame Management
# ---------------------------------------------------------

add_executable(tf_goal_node
  src/tf_goal_node.cpp
)

ament_target_dependencies(tf_goal_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---------------------------------------------------------
# Q2 - Nav2 Controller Plugin
# ---------------------------------------------------------

add_library(simple_controller SHARED
  src/simple_controller.cpp
)

ament_target_dependencies(simple_controller
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  nav2_core
  nav2_costmap_2d
  pluginlib
)

# ---------------------------------------------------------
# Q2 - Controller Demonstration
# ---------------------------------------------------------

add_executable(test_controller_plugin
  src/test_controller_plugin.cpp
)

ament_target_dependencies(test_controller_plugin
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  geometry_msgs
  nav_msgs
)

# ---------------------------------------------------------
# Pluginlib registration
# ---------------------------------------------------------

pluginlib_export_plugin_description_file(
  nav2_core
  plugin.xml
)

# ---------------------------------------------------------
# Install Q1 executable
# ---------------------------------------------------------

install(
  TARGETS tf_goal_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------
# Install Q2 controller plugin
# ---------------------------------------------------------

install(
  TARGETS simple_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ---------------------------------------------------------
# Install Q2 demonstration executable
# ---------------------------------------------------------

install(
  TARGETS test_controller_plugin
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------
# Install plugin description
# ---------------------------------------------------------

install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()