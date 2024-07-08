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

#ifndef TRAPS_EXAMPLE_ROS__EXAMPLE__NODE_HPP_
#define TRAPS_EXAMPLE_ROS__EXAMPLE__NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/node.hpp"
#include "traps_example_ros/msg/example_string.hpp"
#include "traps_example_ros/visibility.hpp"

namespace traps_example_ros::example
{

class Node : public rclcpp::Node
{
public:
  static constexpr auto default_node_name() noexcept {return "example";}

  TRAPS_EXAMPLE_ROS_PUBLIC
  Node(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());

  TRAPS_EXAMPLE_ROS_PUBLIC
  explicit inline Node(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(node_name, "", node_options)
  {
  }

  TRAPS_EXAMPLE_ROS_PUBLIC
  explicit inline Node(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node(this->default_node_name(), "", node_options)
  {
  }

  void republish(traps_example_ros::msg::ExampleString::ConstSharedPtr string_msg);

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params);

private:
  rclcpp::Publisher<traps_example_ros::msg::ExampleString>::SharedPtr republish_string_publisher_;
  rclcpp::Subscription<traps_example_ros::msg::ExampleString>::SharedPtr string_subscription_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    on_set_parameter_call_back_handle_;
};

}  // namespace traps_example_ros::example

#endif  // TRAPS_EXAMPLE_ROS__EXAMPLE__NODE_HPP_
