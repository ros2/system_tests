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

#include <chrono>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "message_fixtures.hpp"

template<typename T>
typename rclcpp::publisher::Publisher<T>::SharedPtr publish(
  rclcpp::Node::SharedPtr node,
  std::vector<typename T::SharedPtr> messages,
  size_t number_of_cycles = 5)
{
  auto start = std::chrono::steady_clock::now();

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = messages.size();
  custom_qos_profile.durability = RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY;

  auto publisher = node->create_publisher<T>(
    std::string("test_qos_message_primitives"), custom_qos_profile);

  rclcpp::WallRate time_between_cycles(1);
  rclcpp::WallRate time_between_messages(10);
  size_t cycle_index = 0;
  // publish all messages up to number_of_cycles times, longer sleep between each cycle
  while (rclcpp::ok() && cycle_index < number_of_cycles) {
    size_t message_index = 0;
    // publish all messages one by one, shorter sleep between each message
    while (rclcpp::ok() && message_index < messages.size()) {
      std::cout << "publishing message #" << (message_index + 1) << std::endl;
      publisher->publish(messages[message_index]);
      ++message_index;
      time_between_messages.sleep();
    }
    ++cycle_index;
    time_between_cycles.sleep();
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "published for " << diff.count() << " seconds" << std::endl;

  return publisher;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(std::string("test_qos_publisher_primitives"));

  auto publisher = publish<test_communication::msg::Primitives>(node, get_messages_primitives(), 1);

  // Wait for a while before exiting
  std::chrono::seconds wait_time(20);

  std::this_thread::sleep_for(wait_time);

  return 0;
}
