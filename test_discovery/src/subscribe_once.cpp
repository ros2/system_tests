// Copyright 2023 Open Source Robotics Foundation, Inc.
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

#include <cstdlib>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/builtins.hpp>

constexpr double kTimeout = 10;

void topic_callback(const test_msgs::msg::Builtins &)
{
  std::cout << "test_discovery: message was received\n" << std::flush;
  std::exit(0);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("subscribe_once");
  auto subscription =
    node->create_subscription<test_msgs::msg::Builtins>("test_topic", 10, topic_callback);

  std::cout << "test_discovery: node successfully created\n" << std::flush;

  auto clock = node->get_clock();
  auto end_time = clock->now() + rclcpp::Duration::from_seconds(kTimeout);
  while (rclcpp::ok() && clock->now() <= end_time) {
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
