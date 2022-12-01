/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#include "beginner_tutorials/publisher_member_function.hpp"

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto serviceCallbackPtr =
        std::bind(&MinimalPublisher::change, this, _1, _2);
    service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
        "change_message", serviceCallbackPtr);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    this->make_transforms();
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

  void MinimalPublisher::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = current_message + std::to_string(count_++);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: " << message.data);
    publisher_->publish(message);
  }

  void MinimalPublisher::change(
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

  void MinimalPublisher::make_transforms()
  {
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";

    t.transform.translation.x = 1;
    t.transform.translation.y = 2;
    t.transform.translation.z = 3;
    t.transform.rotation.x = 1;
    t.transform.rotation.y = 0;
    t.transform.rotation.z = 0;
    t.transform.rotation.w = 0;

    tf_static_broadcaster_->sendTransform(t);
  }