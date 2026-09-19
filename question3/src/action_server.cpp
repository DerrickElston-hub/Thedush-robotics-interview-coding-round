#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "ros2_q3/action/go_to_charger.hpp"

using namespace std::chrono_literals;

class GoToChargerActionServer : public rclcpp::Node
{
public:
  using GoToCharger = ros2_q3::action::GoToCharger;
  using GoalHandleGoToCharger =
    rclcpp_action::ServerGoalHandle<GoToCharger>;

  GoToChargerActionServer()
  : Node("go_to_charger_action_server")
  {
    using namespace std::placeholders;

    action_server_ = rclcpp_action::create_server<GoToCharger>(
      this,
      "go_to_charger",
      std::bind(
        &GoToChargerActionServer::handle_goal,
        this,
        _1,
        _2),
      std::bind(
        &GoToChargerActionServer::handle_cancel,
        this,
        _1),
      std::bind(
        &GoToChargerActionServer::handle_accepted,
        this,
        _1));

    RCLCPP_INFO(
      this->get_logger(),
      "GoToCharger ACTION server ready");
  }

private:

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const GoToCharger::Goal> goal)
  {
    (void)uuid;

    RCLCPP_INFO(
      this->get_logger(),
      "Action goal received: distance = %.1f m",
      goal->distance);

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    (void)goal_handle;

    RCLCPP_INFO(
      this->get_logger(),
      "Action cancellation requested");

    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void handle_accepted(
    const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    std::thread{
      std::bind(
        &GoToChargerActionServer::execute,
        this,
        std::placeholders::_1),
      goal_handle
    }.detach();
  }


  void execute(
    const std::shared_ptr<GoalHandleGoToCharger> goal_handle)
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Executing 30-second charging station task...");

    auto result =
      std::make_shared<GoToCharger::Result>();

    auto feedback =
      std::make_shared<GoToCharger::Feedback>();

    const auto start_time =
      std::chrono::steady_clock::now();

    // Simulate a 30-second task.
    // Distance starts at 30 m and decreases by 2 m every second.
    for (int second = 1; second <= 30; ++second)
    {
      // Check whether the client requested cancellation.
      if (goal_handle->is_canceling())
      {
        result->success = false;
        result->travel_time =
          std::chrono::duration<float>(
            std::chrono::steady_clock::now() - start_time).count();

        result->message = "Goal canceled";

        goal_handle->canceled(result);

        RCLCPP_INFO(
          this->get_logger(),
          "Charging station task canceled");

        return;
      }

      std::this_thread::sleep_for(1s);

      // Calculate remaining distance.
      float remaining =
        std::max(
          0.0f,
          30.0f -
          (2.0f * static_cast<float>(second)));

      // Publish feedback only while distance remains.
      // This prevents repeated 0.0 m feedback messages.
      if (remaining > 0.0f)
      {
        feedback->distance_remaining = remaining;

        goal_handle->publish_feedback(feedback);

        RCLCPP_INFO(
          this->get_logger(),
          "Feedback: distance remaining = %.1f m",
          remaining);
      }
    }

    // Calculate total travel time.
    const auto end_time =
      std::chrono::steady_clock::now();

    const float travel_time =
      std::chrono::duration<float>(
        end_time - start_time).count();

    // Prepare final result.
    result->success = true;
    result->travel_time = travel_time;
    result->message = "Arrived at charging station!";

    goal_handle->succeed(result);

    RCLCPP_INFO(
      this->get_logger(),
      "Charging station task completed");

    RCLCPP_INFO(
      this->get_logger(),
      "Travel time: %.1f s",
      travel_time);
  }


  rclcpp_action::Server<GoToCharger>::SharedPtr action_server_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<GoToChargerActionServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}