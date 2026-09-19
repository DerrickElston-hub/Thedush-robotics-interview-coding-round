#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_q3/srv/go_to_charger.hpp"

using namespace std::chrono_literals;

class ServiceServer : public rclcpp::Node
{
public:
  ServiceServer()
  : Node("go_to_charger_service_server")
  {
    service_ = create_service<ros2_q3::srv::GoToCharger>(
      "go_to_charger_service",
      std::bind(
        &ServiceServer::handle_request,
        this,
        std::placeholders::_1,
        std::placeholders::_2));

    RCLCPP_INFO(
      get_logger(),
      "GoToCharger SERVICE server ready");
  }

private:
  void handle_request(
    const std::shared_ptr<ros2_q3::srv::GoToCharger::Request> request,
    std::shared_ptr<ros2_q3::srv::GoToCharger::Response> response)
  {
    RCLCPP_INFO(
      get_logger(),
      "Service request received: distance = %.1f m",
      request->distance);

    RCLCPP_INFO(
      get_logger(),
      "Simulating 30-second charging station trip...");

    std::this_thread::sleep_for(30s);

    response->success = true;
    response->travel_time = 30.0;
    response->message = "Arrived at charging station";

    RCLCPP_INFO(
      get_logger(),
      "Service task completed");
  }

  rclcpp::Service<ros2_q3::srv::GoToCharger>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<ServiceServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
