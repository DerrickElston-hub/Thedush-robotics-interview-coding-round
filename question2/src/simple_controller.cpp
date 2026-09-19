#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace simple_controller
{

class SimpleController : public nav2_core::Controller
{
public:
  SimpleController() = default;

  ~SimpleController() override = default;

  // ---------------------------------------------------------
  // Configure
  // ---------------------------------------------------------
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    (void)tf;
    (void)costmap_ros;

    node_ = parent.lock();
    name_ = name;

    node_->declare_parameter<double>(
      name_ + ".linear_velocity", 0.2);

    node_->declare_parameter<double>(
      name_ + ".kp", 1.0);

    node_->declare_parameter<double>(
      name_ + ".waypoint_tolerance", 0.3);

    linear_velocity_ =
      node_->get_parameter(
        name_ + ".linear_velocity").as_double();

    kp_ =
      node_->get_parameter(
        name_ + ".kp").as_double();

    waypoint_tolerance_ =
      node_->get_parameter(
        name_ + ".waypoint_tolerance").as_double();

    // Initial speed limit is the configured linear velocity.
    speed_limit_ = linear_velocity_;

    RCLCPP_INFO(
      node_->get_logger(),
      "SimpleController configured");

    RCLCPP_INFO(
      node_->get_logger(),
      "Linear velocity: %.2f m/s",
      linear_velocity_);

    RCLCPP_INFO(
      node_->get_logger(),
      "Kp: %.2f",
      kp_);

    RCLCPP_INFO(
      node_->get_logger(),
      "Waypoint tolerance: %.2f m",
      waypoint_tolerance_);
  }

  // ---------------------------------------------------------
  // Cleanup
  // ---------------------------------------------------------
  void cleanup() override
  {
    plan_.poses.clear();
    visited_.clear();

    node_.reset();

    RCLCPP_INFO(
      rclcpp::get_logger("simple_controller"),
      "SimpleController cleaned up");
  }

  // ---------------------------------------------------------
  // Activate
  // ---------------------------------------------------------
  void activate() override
  {
    if (node_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "SimpleController activated");
    }
  }

  // ---------------------------------------------------------
  // Deactivate
  // ---------------------------------------------------------
  void deactivate() override
  {
    if (node_) {
      RCLCPP_INFO(
        node_->get_logger(),
        "SimpleController deactivated");
    }
  }

  // ---------------------------------------------------------
  // Set plan
  // ---------------------------------------------------------
  void setPlan(
    const nav_msgs::msg::Path & path) override
  {
    plan_ = path;

    // Reset waypoint visited status for the new plan.
    visited_.assign(
      plan_.poses.size(), false);

    RCLCPP_INFO(
      node_->get_logger(),
      "Received plan with %zu poses",
      plan_.poses.size());
  }

  // ---------------------------------------------------------
  // Compute velocity commands
  // ---------------------------------------------------------
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    (void)velocity;
    (void)goal_checker;

    geometry_msgs::msg::TwistStamped cmd;

    cmd.header.stamp = node_->now();
    cmd.header.frame_id = pose.header.frame_id;

    // -------------------------------------------------------
    // No path available
    // -------------------------------------------------------
    if (plan_.poses.empty()) {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;

      return cmd;
    }

    // -------------------------------------------------------
    // Make sure visited list matches the path
    // -------------------------------------------------------
    if (visited_.size() != plan_.poses.size()) {
      visited_.assign(
        plan_.poses.size(), false);
    }

    // -------------------------------------------------------
    // Mark reached waypoints as visited
    // -------------------------------------------------------
    for (size_t i = 0;
         i < plan_.poses.size();
         ++i)
    {
      const double dx =
        plan_.poses[i].pose.position.x -
        pose.pose.position.x;

      const double dy =
        plan_.poses[i].pose.position.y -
        pose.pose.position.y;

      const double distance =
        std::hypot(dx, dy);

      if (distance <= waypoint_tolerance_) {
        visited_[i] = true;
      }
    }

    // -------------------------------------------------------
    // Find nearest unvisited waypoint
    // -------------------------------------------------------
    size_t target_index =
      plan_.poses.size();

    double nearest_distance =
      std::numeric_limits<double>::max();

    for (size_t i = 0;
         i < plan_.poses.size();
         ++i)
    {
      // Ignore already visited waypoints.
      if (visited_[i]) {
        continue;
      }

      const double dx =
        plan_.poses[i].pose.position.x -
        pose.pose.position.x;

      const double dy =
        plan_.poses[i].pose.position.y -
        pose.pose.position.y;

      const double distance =
        std::hypot(dx, dy);

      if (distance < nearest_distance) {
        nearest_distance = distance;
        target_index = i;
      }
    }

    // -------------------------------------------------------
    // All waypoints have been visited
    // -------------------------------------------------------
    if (target_index == plan_.poses.size()) {
      cmd.twist.linear.x = 0.0;
      cmd.twist.angular.z = 0.0;

      RCLCPP_DEBUG(
        node_->get_logger(),
        "All waypoints have been visited");

      return cmd;
    }

    // -------------------------------------------------------
    // Selected target waypoint
    // -------------------------------------------------------
    const auto & target =
      plan_.poses[target_index];

    const double dx =
      target.pose.position.x -
      pose.pose.position.x;

    const double dy =
      target.pose.position.y -
      pose.pose.position.y;

    // -------------------------------------------------------
    // Desired heading toward target
    // -------------------------------------------------------
    const double desired_heading =
      std::atan2(dy, dx);

    // -------------------------------------------------------
    // Current robot heading
    // Quaternion -> yaw
    // -------------------------------------------------------
    const double current_heading =
      tf2::getYaw(pose.pose.orientation);

    // -------------------------------------------------------
    // Calculate heading error
    // -------------------------------------------------------
    double heading_error =
      desired_heading -
      current_heading;

    // -------------------------------------------------------
    // Normalize heading error to [-pi, pi]
    // -------------------------------------------------------
    while (heading_error > M_PI) {
      heading_error -= 2.0 * M_PI;
    }

    while (heading_error < -M_PI) {
      heading_error += 2.0 * M_PI;
    }

    // -------------------------------------------------------
    // Proportional heading controller
    //
    // angular velocity = Kp * heading error
    // linear velocity  = constant configured speed
    // -------------------------------------------------------
    cmd.twist.linear.x =
      speed_limit_;

    cmd.twist.angular.z =
      kp_ * heading_error;

    return cmd;
  }

  // ---------------------------------------------------------
  // Set speed limit
  // ---------------------------------------------------------
  void setSpeedLimit(
    const double & speed_limit,
    const bool & percentage) override
  {
    if (percentage) {

      speed_limit_ =
        linear_velocity_ *
        speed_limit /
        100.0;

    } else {

      speed_limit_ =
        speed_limit;
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "Speed limit updated: %.2f m/s",
      speed_limit_);
  }

private:

  // ROS 2 lifecycle node
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  // Plugin instance name
  std::string name_;

  // Nav2 global plan
  nav_msgs::msg::Path plan_;

  // Tracks which waypoints have been reached
  std::vector<bool> visited_;

  // Controller parameters
  double linear_velocity_{0.2};

  double kp_{1.0};

  double waypoint_tolerance_{0.3};

  // Current speed limit
  double speed_limit_{0.2};
};

}  // namespace simple_controller

// -----------------------------------------------------------
// Pluginlib registration
// -----------------------------------------------------------
PLUGINLIB_EXPORT_CLASS(
  simple_controller::SimpleController,
  nav2_core::Controller)