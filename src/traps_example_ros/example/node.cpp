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

#include "traps_example_ros/example/node.hpp"

#include "traps_example_ros/dynamic_qos.hpp"
#include "traps_example_ros/example.hpp"

namespace traps_example_ros::example
{

Node::Node(
  const std::string & node_name, const std::string & node_namespace,
  const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_namespace, node_options),
  republish_string_publisher_(this->create_publisher<traps_example_ros::msg::ExampleString>(
      std::string(this->get_name()) + "/republish_string", traps_example_ros::dynamic_qos())),
  string_subscription_(this->create_subscription<traps_example_ros::msg::ExampleString>(
      std::string(this->get_name()) + "/string", traps_example_ros::dynamic_qos(),
      [this](traps_example_ros::msg::ExampleString::ConstSharedPtr string_msg) {
        this->republish(string_msg);
      })),
  on_set_parameter_call_back_handle_(
    this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        return this->on_set_parameters_callback(std::move(params));
      }))
{
  this->declare_parameter("parameter_by", "node default");
}

void Node::republish(traps_example_ros::msg::ExampleString::ConstSharedPtr string_msg)
{
  RCLCPP_INFO(this->get_logger(), "republish string : %s", string_msg->data.c_str());
  republish_string_publisher_->publish(Example::example_string_msg_from_ptr(string_msg));
}

rcl_interfaces::msg::SetParametersResult Node::on_set_parameters_callback(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult set_parameters_result_msg;
  set_parameters_result_msg.successful = true;
  for (const auto & param : params) {
    try {
      if (param.get_name() == "parameter_by") {
        [[maybe_unused]] auto parameter_by = param.as_string();
      }
      RCLCPP_INFO(
        this->get_logger(), "Successfully set the value of parameter \"%s\" to \"%s\"",
        param.get_name().c_str(), param.value_to_string().c_str());
    } catch (std::exception & e) {
      set_parameters_result_msg.successful = false;
      set_parameters_result_msg.reason += std::string(e.what()) + "; ";
    }
  }

  return set_parameters_result_msg;
}

}  // namespace traps_example_ros::example

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(traps_example_ros::example::Node)
