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
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/message_fixtures.hpp"

template<typename T>
void publish(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  std::vector<typename T::SharedPtr> messages,
  size_t number_of_cycles = 100)
{
  auto start = std::chrono::steady_clock::now();

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = messages.size();

  auto publisher = node->create_publisher<T>(
    std::string("test/message/") + message_type, custom_qos_profile);

  rclcpp::WallRate cycle_rate(10);
  rclcpp::WallRate message_rate(100);
  size_t cycle_index = 0;
  // publish all messages up to number_of_cycles times, longer sleep between each cycle
  while (rclcpp::ok() && cycle_index < number_of_cycles) {
    size_t message_index = 0;
    // publish all messages one by one, shorter sleep between each message
    while (rclcpp::ok() && message_index < messages.size()) {
      printf("publishing message #%zu\n", message_index + 1);
      publisher->publish(messages[message_index]);
      ++message_index;
      message_rate.sleep();
    }
    ++cycle_index;
    cycle_rate.sleep();
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("published for %f seconds\n", diff.count());
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    fprintf(stderr, "Wrong number of arguments, pass one message type\n");
    return 1;
  }
  rclcpp::init(argc, argv);

  std::string message = argv[1];
  std::string namespace_ = argv[2];
  auto node = rclcpp::Node::make_shared(
    std::string("test_publisher_") + message, namespace_);

  if (message == "Empty") {
    publish<test_msgs::msg::Empty>(node, message, get_messages_empty());
  } else if (message == "Primitives") {
    publish<test_msgs::msg::Primitives>(node, message, get_messages_primitives());
  } else if (message == "StaticArrayPrimitives") {
    publish<test_msgs::msg::StaticArrayPrimitives>(
      node, message, get_messages_static_array_primitives());
  } else if (message == "StaticArrayPrimitivesNested") {
    publish<test_msgs::msg::StaticArrayPrimitivesNested>(
      node, message, get_messages_static_array_primitives_nested());
  } else if (message == "DynamicArrayPrimitives") {
    publish<test_msgs::msg::DynamicArrayPrimitives>(
      node, message, get_messages_dynamic_array_primitives());
  } else if (message == "DynamicArrayPrimitivesNested") {
    publish<test_msgs::msg::DynamicArrayPrimitivesNested>(
      node, message, get_messages_dynamic_array_primitives_nested());
  } else if (message == "DynamicArrayStaticArrayPrimitivesNested") {
    publish<test_msgs::msg::DynamicArrayStaticArrayPrimitivesNested>(
      node, message, get_messages_dynamic_array_static_array_primitives_nested());
  } else if (message == "BoundedArrayPrimitives") {
    publish<test_msgs::msg::BoundedArrayPrimitives>(
      node, message, get_messages_bounded_array_primitives());
  } else if (message == "BoundedArrayPrimitivesNested") {
    publish<test_msgs::msg::BoundedArrayPrimitivesNested>(
      node, message, get_messages_bounded_array_primitives_nested());
  } else if (message == "Nested") {
    publish<test_msgs::msg::Nested>(node, message, get_messages_nested());
  } else if (message == "DynamicArrayNested") {
    publish<test_msgs::msg::DynamicArrayNested>(
      node, message, get_messages_dynamic_array_nested());
  } else if (message == "BoundedArrayNested") {
    publish<test_msgs::msg::BoundedArrayNested>(
      node, message, get_messages_bounded_array_nested());
  } else if (message == "StaticArrayNested") {
    publish<test_msgs::msg::StaticArrayNested>(
      node, message, get_messages_static_array_nested());
  } else if (message == "Builtins") {
    publish<test_msgs::msg::Builtins>(node, message, get_messages_builtins());
  } else {
    fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
