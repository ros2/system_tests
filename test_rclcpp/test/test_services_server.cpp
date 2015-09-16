// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <rclcpp/rclcpp.hpp>

#include <test_rclcpp/srv/add_two_ints.hpp>

void handle_add_two_ints(
  const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_server");

  node->create_service<test_rclcpp::srv::AddTwoInts>("add_two_ints", handle_add_two_ints);

  rclcpp::spin(node);

  return 0;
}
