// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "test_rclcpp/srv/add_two_ints.hpp"

void handle_add_two_ints(
  const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
{
  printf("responding to request\n");
  std::cout.flush();
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("client_scope_consistency_regression_test_server");

  // Replicate the settings that caused https://github.com/ros2/system_tests/issues/153
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  auto service = node->create_service<test_rclcpp::srv::AddTwoInts>(
    "client_scope", handle_add_two_ints, qos);

  rclcpp::WallRate loop_rate(30);
  try {
    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
      loop_rate.sleep();
    }
  } catch (const rclcpp::exceptions::RCLError & ex) {
    RCLCPP_ERROR(node->get_logger(), "failed with %s", ex.what());
  }
  rclcpp::shutdown();
  return 0;
}
