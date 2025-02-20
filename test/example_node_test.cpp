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

#ifndef EXAMPLE_NODE_TEST_HPP_
#define EXAMPLE_NODE_TEST_HPP_

#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "traps_example_ros/example_node.hpp"
#include "traps_example_ros/example_tester_node.hpp"

TEST(example_node, string_republish)
{
  namespace ter = traps_example_ros;

  // create nodes
  auto node = std::make_shared<ter::ExampleNode>();
  auto tester_node = std::make_shared<ter::ExampleTesterNode>();

  // publish
  const std::string pub_string = "example_test";
  tester_node->pub_string(pub_string);

  // spin
  rclcpp::spin_some(node);
  rclcpp::spin_some(tester_node);

  // check
  const auto sub_strings = tester_node->sub_strings();
  ASSERT_TRUE(sub_strings.size() == 1)
    << "Subscribed count doesn't 1 (subscribed count: " << sub_strings.size() << ")";
  ASSERT_TRUE(sub_strings.front() == pub_string)
    << "Subscribed value is not \"" << pub_string << "\" (subscribed value: " << sub_strings.front()
    << ")";
}

#endif  // EXAMPLE_NODE_TEST_HPP_
