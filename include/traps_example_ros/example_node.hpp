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

#ifndef TRAPS_EXAMPLE_ROS__EXAMPLE_NODE_HPP_
#define TRAPS_EXAMPLE_ROS__EXAMPLE_NODE_HPP_

#include <deque>
#include <string>
#include <utility>
#include <vector>

#include "fmt/format.h"
#include "rclcpp/node.hpp"
#include "traps_example_ros/msg/example_string.hpp"
#include "traps_example_ros/visibility.hpp"

namespace traps_example_ros
{

class ExampleNode : public rclcpp::Node
{
public:
  static constexpr auto kDefaultNodeName = "traps_example";

  TRAPS_EXAMPLE_ROS_PUBLIC
  inline ExampleNode(
    const std::string & node_name, const std::string & node_namespace,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : rclcpp::Node(node_name, node_namespace, node_options)
  {
    // Create a pub/sub options
    static const auto qos_overriding_options = rclcpp::QosOverridingOptions{
      rclcpp::QosPolicyKind::Depth, rclcpp::QosPolicyKind::Durability,
      rclcpp::QosPolicyKind::History, rclcpp::QosPolicyKind::Reliability};
    static const auto pub_options = [&] {
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = qos_overriding_options;
        return options;
      }();
    static const auto sub_options = [&] {
        rclcpp::SubscriptionOptions options;
        options.qos_overriding_options = qos_overriding_options;
        return options;
      }();

    // create pub/sub
    republish_string_pub_ = this->create_publisher<traps_example_ros::msg::ExampleString>(
      "republish_string", rclcpp::QoS(1).best_effort(), pub_options);
    string_sub_ = this->create_subscription<traps_example_ros::msg::ExampleString>(
      "string", rclcpp::QoS(1).best_effort(),
      [this](traps_example_ros::msg::ExampleString::UniquePtr string_msg) {
        this->republish(std::move(string_msg));
      },
      sub_options);

    // set parameter callback
    param_callback_handle_ =
      this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return this->on_set_parameters_callback(params);
      });

    // declare parameter
    this->declare_parameter("string", "node default");
  }

  TRAPS_EXAMPLE_ROS_PUBLIC
  explicit inline ExampleNode(
    const std::string & node_name, const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : ExampleNode(node_name, "", node_options)
  {
  }

  TRAPS_EXAMPLE_ROS_PUBLIC
  explicit inline ExampleNode(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : ExampleNode(kDefaultNodeName, "", node_options)
  {
  }

private:
  void republish(traps_example_ros::msg::ExampleString::UniquePtr string_msg)
  {
    // If you do not use subscribed values, use ConstSharedPtr instead of UniquePtr
    RCLCPP_INFO(this->get_logger(), "republish string: \"%s\"", string_msg->data.c_str());
    republish_string_pub_->publish(std::move(string_msg));
  }

  rcl_interfaces::msg::SetParametersResult on_set_parameters_callback(
    const std::vector<rclcpp::Parameter> & params)
  {
    // set parameters
    std::deque<std::string> error_strs;
    for (const auto & param : params) {
      // set each parameter
      if (param.get_name() == "string") {
        const auto str = param.as_string();
        RCLCPP_INFO(this->get_logger(), "set string to \"%s\"", str.c_str());
      } else {
        error_strs.push_back(
          fmt::format("unknown parameter({}: {})", param.get_name(), param.value_to_string()));
      }
    }

    // print errors
    for (const auto & error_str : error_strs) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set parameter: %s", error_str.c_str());
    }

    // return value
    rcl_interfaces::msg::SetParametersResult set_parameters_result_msg;
    set_parameters_result_msg.successful = error_strs.empty();
    set_parameters_result_msg.reason = fmt::format("{}", fmt::join(error_strs, "; "));
    return set_parameters_result_msg;
  }

  // pub/sub
  rclcpp::Publisher<traps_example_ros::msg::ExampleString>::SharedPtr republish_string_pub_;
  rclcpp::Subscription<traps_example_ros::msg::ExampleString>::SharedPtr string_sub_;

  // parameter
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace traps_example_ros

#endif  // TRAPS_EXAMPLE_ROS__EXAMPLE_NODE_HPP_
