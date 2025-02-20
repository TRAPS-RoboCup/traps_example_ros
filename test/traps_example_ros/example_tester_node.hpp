// Copyright 2025 TRAPS
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

#ifndef TRAPS_EXAMPLE_ROS__EXAMPLE_TESTER_NODE_HPP_
#define TRAPS_EXAMPLE_ROS__EXAMPLE_TESTER_NODE_HPP_

#include <list>
#include <memory>
#include <string>
#include <utility>

#include "gtest/gtest.h"
#include "rclcpp/node.hpp"
#include "traps_example_ros/example_node.hpp"
#include "traps_example_ros/msg/example_string.hpp"

namespace traps_example_ros
{

class ExampleTesterNode : public rclcpp::Node
{
public:
  ExampleTesterNode()
  : Node("traps_example_node_tester"),
    string_sub_(create_subscription<traps_example_ros::msg::ExampleString>(
        "republish_string", rclcpp::QoS(1).best_effort(),
        [this](traps_example_ros::msg::ExampleString::ConstSharedPtr string_msg) {
          set_string(string_msg);
        })),
    string_pub_(
      create_publisher<traps_example_ros::msg::ExampleString>(
        "string", rclcpp::QoS(1).best_effort()))
  {
  }

  void pub_string(const std::string & string)
  {
    auto string_msg = std::make_unique<traps_example_ros::msg::ExampleString>();
    string_msg->data = string;
    string_pub_->publish(std::move(string_msg));
  }

  const std::list<std::string> & sub_strings() const {return sub_strings_;}

private:
  void set_string(msg::ExampleString::ConstSharedPtr string_msg)
  {
    sub_strings_.push_back(string_msg->data);
  }

  // string buffer
  std::list<std::string> sub_strings_;

  // pub/sub
  rclcpp::Subscription<traps_example_ros::msg::ExampleString>::SharedPtr string_sub_;
  rclcpp::Publisher<traps_example_ros::msg::ExampleString>::SharedPtr string_pub_;
};

}  // namespace traps_example_ros

#endif  // TRAPS_EXAMPLE_ROS__EXAMPLE_TESTER_NODE_HPP_
