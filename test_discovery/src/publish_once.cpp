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

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <test_msgs/msg/builtins.hpp>


constexpr double kMaxDiscoveryTime = 10;


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("publish_once");
  auto publisher = node->create_publisher<test_msgs::msg::Builtins>("test_topic", 10);

  auto clock = node->get_clock();

  auto end_time = clock->now() + rclcpp::Duration::from_seconds(kMaxDiscoveryTime);
  while (rclcpp::ok() && publisher->get_subscription_count() == 0) {
    if (clock->now() >= end_time) {
      return 0;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  publisher->publish(test_msgs::msg::Builtins());

  // Do nothing until killed.
  rclcpp::spin(node);
  return 0;
}
