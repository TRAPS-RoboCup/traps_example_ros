// Copyright 2024 TRAPS
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "traps_example_ros/dynamic_qos.hpp"
#include "traps_example_ros/example/node.hpp"
#include "traps_example_ros/msg/example_string.hpp"

TEST(example, string_republish)
{
  auto test_node = std::make_shared<rclcpp::Node>("test");
  traps_example_ros::msg::ExampleString::ConstSharedPtr republish_msg;
  auto example_string_subscription =
    test_node->create_subscription<traps_example_ros::msg::ExampleString>(
    "example/republish_string", traps_example_ros::dynamic_qos(),
    [&republish_msg,
    &test_node](traps_example_ros::msg::ExampleString::ConstSharedPtr republish_msg_arg) {
      republish_msg = std::move(republish_msg_arg);
    });
  auto example_string_publisher =
    test_node->create_publisher<traps_example_ros::msg::ExampleString>(
    "example/string", traps_example_ros::dynamic_qos());

  auto example_node = std::make_shared<traps_example_ros::example::Node>();

  traps_example_ros::msg::ExampleString example_string_msg;
  example_string_msg.data = "example_test";
  example_string_publisher->publish(example_string_msg);

  rclcpp::spin_some(test_node);     // publish "example/string"
  rclcpp::spin_some(example_node);  // subscribe "example/string" & publish "example/string"
  rclcpp::spin_some(test_node);     // subscribe "example/string"

  ASSERT_TRUE(republish_msg) << "Couldn't subscribe \"example/republish_string\"";
  if (!republish_msg) {
    return;
  }
  EXPECT_EQ(republish_msg->data, "example_test")
    << "The value that subscribed to \"example/republish_string\" was " << republish_msg->data
    << ", but it is actually " << example_string_msg.data;
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
