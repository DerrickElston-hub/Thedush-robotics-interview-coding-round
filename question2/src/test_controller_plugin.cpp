#include <cmath>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"

#include "pluginlib/class_loader.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>(
      "q2_controller_demo");

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  RCLCPP_INFO(
    node->get_logger(),
    "Q2 Nav2 Controller Plugin Demo");

  RCLCPP_INFO(
    node->get_logger(),
    "========================================");

  try {

    // -------------------------------------------------------
    // Load controller through pluginlib
    // -------------------------------------------------------

    pluginlib::ClassLoader<nav2_core::Controller> loader(
      "nav2_core",
      "nav2_core::Controller");

    auto controller =
      loader.createSharedInstance(
        "simple_controller::SimpleController");

    RCLCPP_INFO(
      node->get_logger(),
      "Plugin loaded successfully");

    // -------------------------------------------------------
    // Configure plugin
    // -------------------------------------------------------

    controller->configure(
      node,
      "SimpleController",
      nullptr,
      nullptr);

    controller->activate();

    // -------------------------------------------------------
    // Create a path with two waypoints
    // -------------------------------------------------------

    nav_msgs::msg::Path path;

    path.header.frame_id = "map";

    geometry_msgs::msg::PoseStamped waypoint1;
    waypoint1.header.frame_id = "map";

    waypoint1.pose.position.x = 2.0;
    waypoint1.pose.position.y = 1.0;

    waypoint1.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped waypoint2;
    waypoint2.header.frame_id = "map";

    waypoint2.pose.position.x = 1.0;
    waypoint2.pose.position.y = 3.0;

    waypoint2.pose.orientation.w = 1.0;

    path.poses.push_back(waypoint1);
    path.poses.push_back(waypoint2);

    // Give path to controller.
    controller->setPlan(path);

    RCLCPP_INFO(
      node->get_logger(),
      "Path sent to controller");

    RCLCPP_INFO(
      node->get_logger(),
      "Waypoint 1: (2.0, 1.0)");

    RCLCPP_INFO(
      node->get_logger(),
      "Waypoint 2: (1.0, 3.0)");

    // -------------------------------------------------------
    // Robot pose: (0, 0), yaw = 0
    // -------------------------------------------------------

    geometry_msgs::msg::PoseStamped robot_pose;

    robot_pose.header.frame_id = "map";

    robot_pose.pose.position.x = 0.0;
    robot_pose.pose.position.y = 0.0;

    // Quaternion for yaw = 0
    robot_pose.pose.orientation.x = 0.0;
    robot_pose.pose.orientation.y = 0.0;
    robot_pose.pose.orientation.z = 0.0;
    robot_pose.pose.orientation.w = 1.0;

    geometry_msgs::msg::Twist current_velocity;

    // -------------------------------------------------------
    // Compute command
    // -------------------------------------------------------

    auto cmd =
      controller->computeVelocityCommands(
        robot_pose,
        current_velocity,
        nullptr);

    RCLCPP_INFO(
      node->get_logger(),
      "----------------------------------------");

    RCLCPP_INFO(
      node->get_logger(),
      "Robot pose: (0.0, 0.0), yaw = 0 deg");

    RCLCPP_INFO(
      node->get_logger(),
      "Nearest unvisited waypoint: (2.0, 1.0)");

    RCLCPP_INFO(
      node->get_logger(),
      "Expected heading error: %.3f rad",
      std::atan2(1.0, 2.0));

    RCLCPP_INFO(
      node->get_logger(),
      "Computed linear.x: %.3f m/s",
      cmd.twist.linear.x);

    RCLCPP_INFO(
      node->get_logger(),
      "Computed angular.z: %.3f rad/s",
      cmd.twist.angular.z);

    // -------------------------------------------------------
    // Move robot to waypoint 1
    // -------------------------------------------------------

    robot_pose.pose.position.x = 2.0;
    robot_pose.pose.position.y = 1.0;

    auto cmd2 =
      controller->computeVelocityCommands(
        robot_pose,
        current_velocity,
        nullptr);

    RCLCPP_INFO(
      node->get_logger(),
      "----------------------------------------");

    RCLCPP_INFO(
      node->get_logger(),
      "Robot reached waypoint 1");

    RCLCPP_INFO(
      node->get_logger(),
      "Waypoint 1 is now marked visited");

    RCLCPP_INFO(
      node->get_logger(),
      "Next nearest unvisited waypoint: (1.0, 3.0)");

    RCLCPP_INFO(
      node->get_logger(),
      "Computed linear.x: %.3f m/s",
      cmd2.twist.linear.x);

    RCLCPP_INFO(
      node->get_logger(),
      "Computed angular.z: %.3f rad/s",
      cmd2.twist.angular.z);

    RCLCPP_INFO(
      node->get_logger(),
      "----------------------------------------");

    controller->deactivate();
    controller->cleanup();

    RCLCPP_INFO(
      node->get_logger(),
      "Q2 controller demonstration completed successfully");

  }
  catch (const std::exception & e) {

    RCLCPP_ERROR(
      node->get_logger(),
      "Q2 demo failed: %s",
      e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();

  return 0;
}
