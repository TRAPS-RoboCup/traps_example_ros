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

#include "traps_example_ros/example.hpp"

namespace traps_example_ros
{

traps_example_ros::msg::ExampleString Example::example_string_msg_from_ptr(
  traps_example_ros::msg::ExampleString::ConstSharedPtr example_string_msg)
{
  return *example_string_msg;
}

}  // namespace traps_example_ros
