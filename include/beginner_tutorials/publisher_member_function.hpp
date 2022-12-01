// MIT License

// Copyright (c) 2022 Pavan Mantripragada

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once
#include <chrono>
#include <example_interfaces/srv/add_two_ints.hpp>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
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
  /**
   * @brief method for
   * publishing static transform
   */
  void make_transforms();
  /** @brief timer pointer */
  rclcpp::TimerBase::SharedPtr timer_;
  /** @brief publisher pointer */
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /** @brief server pointer */
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
  /** @brief tf broadcaster pointer */
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  /** @brief count of no. of msgs published */
  size_t count_;
};
