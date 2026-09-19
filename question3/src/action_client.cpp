#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2_q3/action/go_to_charger.hpp"

using namespace std::chrono_literals;

class ActionClient : public rclcpp::Node
{
public:
  using GoToCharger = ros2_q3::action::GoToCharger;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GoToCharger>;

  ActionClient()
  : Node("go_to_charger_action_client")
  {
    action_client_ =
      rclcpp_action::create_client<GoToCharger>(
        this,
        "go_to_charger");

    timer_ =
      create_wall_timer(
        500ms,
        std::bind(
          &ActionClient::send_goal,
          this));

    RCLCPP_INFO(
      get_logger(),
      "GoToCharger ACTION client ready");
  }

private:
  void send_goal()
  {
    timer_->cancel();

    if (!action_client_->wait_for_action_server(2s)) {

      RCLCPP_ERROR(
        get_logger(),
        "Action server not available");

      rclcpp::shutdown();
      return;
    }

    GoToCharger::Goal goal;

    goal.distance = 30.0;

    RCLCPP_INFO(
      get_logger(),
      "Sending GoToCharger action goal...");

    auto send_goal_options =
      rclcpp_action::Client<GoToCharger>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(
        &ActionClient::goal_response_callback,
        this,
        std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(
        &ActionClient::feedback_callback,
        this,
        std::placeholders::_1,
        std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(
        &ActionClient::result_callback,
        this,
        std::placeholders::_1);

    action_client_->async_send_goal(
      goal,
      send_goal_options);
  }

  void goal_response_callback(
    const GoalHandle::SharedPtr & goal_handle)
  {
    if (!goal_handle) {

      RCLCPP_ERROR(
        get_logger(),
        "Action goal was rejected");

      rclcpp::shutdown();
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Goal accepted - receiving feedback...");
  }

  void feedback_callback(
    GoalHandle::SharedPtr,
    const std::shared_ptr<const GoToCharger::Feedback> feedback)
  {
    RCLCPP_INFO(
      get_logger(),
      "Distance remaining: %.1f m",
      feedback->distance_remaining);
  }

  void result_callback(
    const GoalHandle::WrappedResult & result)
  {
    if (result.code ==
      rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(
        get_logger(),
        "Result: %s Travel time: %.1f s",
        result.result->message.c_str(),
        result.result->travel_time);
    }
    else {
      RCLCPP_ERROR(
        get_logger(),
        "Action did not complete successfully");
    }

    rclcpp::shutdown();
  }

  rclcpp_action::Client<GoToCharger>::SharedPtr action_client_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ActionClient>();

  rclcpp::spin(node);

  return 0;
}
