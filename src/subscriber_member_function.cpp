/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;
/**
 * @brief A Subscriber
 * class
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Constructor
   * for MinimalSubscriber
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Call back function
   * to print the heard msg when
   * something is published on
   * the topic
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: " << msg.data);
  }
  /** @brief subscriber pointer */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Subscriber died!");
  return 0;
}
