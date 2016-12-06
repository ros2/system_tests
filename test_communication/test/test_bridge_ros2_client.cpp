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

#include "rclcpp/rclcpp.hpp"
#include <diagnostic_msgs/srv/self_test.hpp>

using namespace diagnostic_msgs::srv;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("ros1_bridge_test_client");
  auto client = node->create_client<SelfTest>("ros1_bridge_test");
  auto request = std::make_shared<SelfTest::Request>();

  while (!client->wait_for_service(4_s)) {
    if (!rclcpp::ok()) {
      printf("add_two_ints_client was interrupted while waiting for the service. Exiting.\n");
      return 0;
    }
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
  {
    auto response = future.get();
    if (response->id != "ros1") {
      throw std::runtime_error("Expected a response from ROS1");
    }
  } else {
    throw std::runtime_error("Failed to call service add_two_ints");
  }

  rclcpp::shutdown();
  return 0;
}
