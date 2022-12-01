/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#pragma once
#include <chrono>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

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
  MinimalPublisher();
  /** @brief current message that will be published */
  std::string current_message = "My Custom String Message :) ";
 
 private:
  /**
   * @brief Call back function
   * to log the published messag
   * along with the count
   */
  void timer_callback();
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
      std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response);
  void make_transforms();
  /** @brief timer pointer */
  rclcpp::TimerBase::SharedPtr timer_;
  /** @brief publisher pointer */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /** @brief server pointer */
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  /** @brief count of no. of msgs published */
  size_t count_;
};