/* Copyright 2022
 * Author
 * Pavan Mantripragada
 */

#include <gtest/gtest.h>
#include <stdlib.h>
#include "beginner_tutorials/publisher_member_function.hpp"

TEST(publisherTest, initialMessageCheck) {
  rclcpp::init(0, nullptr);
  MinimalPublisher pub;
  rclcpp::shutdown();
  EXPECT_TRUE(pub.current_message == "My Custom String Message :) ");
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}