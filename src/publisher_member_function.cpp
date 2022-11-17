/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#include <chrono>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;
/**
 * @brief A publisher cum server
 * class
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   * for MinimalPublisher
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::change, this, _1, _2);
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "change_message", serviceCallbackPtr);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /**
   * @brief Call back function
   * to log the published messag
   * along with the count
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = current_message + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }
  /**
   * @brief service callback
   * for responding to th request
   *
   * @param request request of the service message
   * @param response response to the service message
   */
  void change(
      const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request>
          request,
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    response->sum = request->a + request->b;
    this->current_message =
        std::to_string(response->sum) + " issa sum of two ints! ";
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "Incoming request\na: " << std::to_string(request->a)
                                << "\nb: " << std::to_string(request->b));
    RCLCPP_DEBUG_STREAM(
        this->get_logger(),
        "sending back response: " << std::to_string(response->sum));
  }
  /** @brief current message that will be published */
  std::string current_message = "My Custom String Message :) ";
  /** @brief timer pointer */
  rclcpp::TimerBase::SharedPtr timer_;
  /** @brief publisher pointer */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /** @brief server pointer */
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  /** @brief count of no. of msgs published */
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Publisher died!");
  return 0;
}
