/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#include "beginner_tutorials/publisher_member_function.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Publisher died!");
  return 0;
}
