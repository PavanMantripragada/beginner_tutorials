/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#include <chrono>
#include <cstdlib>
#include <memory>

#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node =
      rclcpp::Node::make_shared("message_change_client");
  /** @brief service client pointer */
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =
      node->create_client<example_interfaces::srv::AddTwoInts>(
          "change_message");

  auto request =
      std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
  node->declare_parameter("A", 0);
  node->declare_parameter("B", 0);
  int A = node->get_parameter("A").get_parameter_value().get<int>();
  int B = node->get_parameter("B").get_parameter_value().get<int>();
  request->a = A;
  request->b = B;

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR_STREAM(
          rclcpp::get_logger("rclcpp"),
          "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "Sum: " << std::to_string(result.get()->sum));
  } else {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
