#include <chrono>
#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/rclcpp.hpp"

#include "tf2/LinearMath/Transform.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

class GoalTfNode : public rclcpp::Node
{
public:
  GoalTfNode()
  : Node("goal_tf_node"),
    goal_received_(false),
    odom_received_(false)
  {
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // ----------------------------------------------------------
    // RViz 2D Goal Pose
    // ----------------------------------------------------------
    goal_sub_ =
      create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose",
        10,
        std::bind(
          &GoalTfNode::goalCallback,
          this,
          std::placeholders::_1));

    // ----------------------------------------------------------
    // Odometry
    // ----------------------------------------------------------
    odom_sub_ =
      create_subscription<nav_msgs::msg::Odometry>(
        "/odom",
        10,
        std::bind(
          &GoalTfNode::odomCallback,
          this,
          std::placeholders::_1));

    // ----------------------------------------------------------
    // TF tree:
    //
    //       map
    //        |
    //       odom
    //        |
    //    base_link
    // ----------------------------------------------------------

    map_odom_.header.frame_id = "map";
    map_odom_.child_frame_id = "odom";

    odom_base_.header.frame_id = "odom";
    odom_base_.child_frame_id = "base_link";

    /*
     * Initial odom -> base_link value.
     *
     * This is only used until the first /odom message arrives.
     * It gives us three visible frame locations for the initial
     * RViz demonstration.
     */
    odom_base_.transform.translation.x = 1.0;
    odom_base_.transform.translation.y = 0.5;
    odom_base_.transform.translation.z = 0.0;

    odom_base_.transform.rotation.x = 0.0;
    odom_base_.transform.rotation.y = 0.0;
    odom_base_.transform.rotation.z = 0.0;
    odom_base_.transform.rotation.w = 1.0;

    // Initially map and odom coincide.
    setIdentity(map_odom_.transform);

    // ----------------------------------------------------------
    // Broadcast TF at 10 Hz
    // ----------------------------------------------------------
    timer_ =
      create_wall_timer(
        100ms,
        std::bind(
          &GoalTfNode::broadcastTf,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Q1 TF node started");

    RCLCPP_INFO(
      get_logger(),
      "TF tree: map -> odom -> base_link");

    RCLCPP_INFO(
      get_logger(),
      "Waiting for /goal_pose and /odom");
  }

private:

  // ------------------------------------------------------------
  // Set a transform to identity
  // ------------------------------------------------------------
  void setIdentity(
    geometry_msgs::msg::Transform & transform)
  {
    transform.translation.x = 0.0;
    transform.translation.y = 0.0;
    transform.translation.z = 0.0;

    transform.rotation.x = 0.0;
    transform.rotation.y = 0.0;
    transform.rotation.z = 0.0;
    transform.rotation.w = 1.0;
  }

  // ------------------------------------------------------------
  // RViz goal callback
  // ------------------------------------------------------------
  void goalCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != "map") {
      RCLCPP_WARN(
        get_logger(),
        "Goal frame is '%s', expected 'map'",
        msg->header.frame_id.c_str());

      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);

    goal_pose_ = *msg;
    goal_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Goal received: x=%.2f, y=%.2f",
      goal_pose_.pose.position.x,
      goal_pose_.pose.position.y);
  }

  // ------------------------------------------------------------
  // Odometry callback
  // ------------------------------------------------------------
  void odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    /*
     * The odometry gives the current:
     *
     *     odom -> base_link
     *
     * pose.
     *
     * We do NOT modify it.
     */
    odom_base_.transform.translation.x =
      msg->pose.pose.position.x;

    odom_base_.transform.translation.y =
      msg->pose.pose.position.y;

    odom_base_.transform.translation.z =
      msg->pose.pose.position.z;

    odom_base_.transform.rotation =
      msg->pose.pose.orientation;

    odom_received_ = true;

    RCLCPP_INFO(
      get_logger(),
      "Odometry updated: x=%.2f, y=%.2f",
      msg->pose.pose.position.x,
      msg->pose.pose.position.y);
  }

  // ------------------------------------------------------------
  // TF calculation and broadcasting
  // ------------------------------------------------------------
  void broadcastTf()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    const auto now = get_clock()->now();

    /*
     * Once a goal exists, calculate the localization correction.
     *
     * We want:
     *
     *     T_map_base = T_goal
     *
     * and we already know:
     *
     *     T_map_base
     *       =
     *     T_map_odom * T_odom_base
     *
     * Therefore:
     *
     *     T_map_odom
     *       =
     *     T_goal * inverse(T_odom_base)
     *
     * This is the important localization calculation.
     */
    if (goal_received_) {

      tf2::Transform goal_tf;
      tf2::Transform odom_base_tf;

      // Desired base_link pose in map frame.
      tf2::fromMsg(
        goal_pose_.pose,
        goal_tf);

      // Current odometry pose.
      tf2::fromMsg(
        odom_base_.transform,
        odom_base_tf);

      // Calculate map -> odom.
      tf2::Transform map_odom_tf =
        goal_tf * odom_base_tf.inverse();

      map_odom_.transform =
        tf2::toMsg(map_odom_tf);
    }

    map_odom_.header.stamp = now;
    odom_base_.header.stamp = now;

    // Broadcast map -> odom.
    tf_broadcaster_->sendTransform(map_odom_);

    // Broadcast odom -> base_link.
    tf_broadcaster_->sendTransform(odom_base_);
  }

  // ------------------------------------------------------------
  // Members
  // ------------------------------------------------------------

  std::unique_ptr<tf2_ros::TransformBroadcaster>
    tf_broadcaster_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    goal_sub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
    odom_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::PoseStamped goal_pose_;

  geometry_msgs::msg::TransformStamped map_odom_;

  geometry_msgs::msg::TransformStamped odom_base_;

  bool goal_received_;
  bool odom_received_;

  std::mutex mutex_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<GoalTfNode>());

  rclcpp::shutdown();

  return 0;
}