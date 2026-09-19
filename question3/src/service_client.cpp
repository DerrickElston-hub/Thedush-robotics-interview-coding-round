#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_q3/srv/go_to_charger.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<rclcpp::Node>(
      "go_to_charger_service_client");

  auto client =
    node->create_client<ros2_q3::srv::GoToCharger>(
      "go_to_charger_service");

  RCLCPP_INFO(
    node->get_logger(),
    "Requesting charging station trip via SERVICE...");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      rclcpp::shutdown();
      return 0;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "Waiting for service server...");
  }

  auto request =
    std::make_shared<ros2_q3::srv::GoToCharger::Request>();

  request->distance = 30.0;

  RCLCPP_INFO(
    node->get_logger(),
    "Waiting for response (5s timeout)...");

  auto future =
    client->async_send_request(request);

  auto status =
    future.wait_for(5s);

  if (status == std::future_status::ready) {

    auto response = future.get();

    RCLCPP_INFO(
      node->get_logger(),
      "Service completed successfully");

    RCLCPP_INFO(
      node->get_logger(),
      "Result: %s",
      response->message.c_str());

    RCLCPP_INFO(
      node->get_logger(),
      "Travel time: %.1f s",
      response->travel_time);

  } else {

    RCLCPP_ERROR(
      node->get_logger(),
      "TIMED OUT - service did not respond within 5 seconds!");
  }

  rclcpp::shutdown();

  return 0;
}
