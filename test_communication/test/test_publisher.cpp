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
int publish(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  std::vector<typename T::Ptr> messages,
  size_t number_of_cycles = 5)
{
  auto start = std::chrono::steady_clock::now();

  auto publisher = node->create_publisher<T>(
    std::string("test_message_") + message_type, messages.size());

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

  return 0;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    fprintf(stderr, "Wrong number of arguments, pass one message type\n");
    return 1;
  }

  std::string message = argv[1];
  auto node = rclcpp::Node::make_shared(std::string("test_publisher_") + message);

  int rc;
  if (message == "empty") {
    rc = publish<test_communication::msg::Empty>(node, message, get_messages_empty());
  } else if (message == "primitives") {
    rc = publish<test_communication::msg::Primitives>(node, message, get_messages_primitives());
  } else if (message == "staticarrayprimitives") {
    rc = publish<test_communication::msg::StaticArrayPrimitives>(node, message, get_messages_static_array_primitives());
  } else if (message == "dynamicarrayprimitives") {
    rc = publish<test_communication::msg::DynamicArrayPrimitives>(node, message, get_messages_dynamic_array_primitives());
  } else if (message == "nested") {
    rc = publish<test_communication::msg::Nested>(node, message, get_messages_nested());
  } else if (message == "builtins") {
    rc = publish<test_communication::msg::Builtins>(node, message, get_messages_builtins());
  } else {
    fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
    return 1;
  }
  return rc;
}
