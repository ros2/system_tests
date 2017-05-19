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
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "message_fixtures.hpp"

template<typename T>
int8_t attempt_publish(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  std::vector<typename T::SharedPtr> messages,
  size_t number_of_cycles = 100)
{
  auto start = std::chrono::steady_clock::now();

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = messages.size();

  // std::cout << "creating publisher" << std::endl;
  auto publisher = node->create_publisher<T>(
    topic_name, custom_qos_profile);
  // std::cout << "publisher created" << std::endl;

  rclcpp::WallRate cycle_rate(10);
  rclcpp::WallRate message_rate(100);
  size_t cycle_index = 0;
  // std::cout << "rclcpp::ok?" << rclcpp::ok() << std::endl;
  // std::cout << "cycle_index < number_of_cycles" << (cycle_index < number_of_cycles) << std::endl;
  // std::cout << "cycle_index:'" << cycle_index << "' number_of_cycles:'" << number_of_cycles << "'" << std::endl;
  // publish all messages up to number_of_cycles times, longer sleep between each cycle
  while (rclcpp::ok() && cycle_index < number_of_cycles) {
    // std::cout << "in while" << std::endl;
    size_t message_index = 0;
    // publish all messages one by one, shorter sleep between each message
    while (rclcpp::ok() && message_index < messages.size()) {
      std::cout << "publishing message #" << (message_index + 1) << std::endl;
      publisher->publish(messages[message_index]);
      ++message_index;
      message_rate.sleep();
    }
    ++cycle_index;
    cycle_rate.sleep();
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "published for " << diff.count() << " seconds" << std::endl;

  return 0;
}

// TODO(mikaelarguedas) separate should_throw on node creation and
// publisher creation to test access control
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 4) {
    fprintf(
      stderr,
      "Wrong number of arguments,\n"
      "pass a node name, a topic name and a should_throw boolean\n");
    return 1;
  }
  std::string node_name = argv[1];
  std::string topic_name = argv[2];
  bool should_throw = ((0 == strcmp(argv[3], "false")) || (0 == strcmp(argv[3], "0"))) ? false : true;
  fprintf(stderr, "should_throw is:'%d'\n", should_throw);
  std::shared_ptr<rclcpp::node::Node> node = nullptr;
  try {
    node = rclcpp::Node::make_shared(node_name);
  } catch (std::runtime_error & e){
    if (should_throw) {
      fprintf(stderr, "throw exception as expected");
      return 1;
    } else {
      fprintf(stderr, "should not have thrown!");
      return 0;
    }
  }
  fprintf(stderr, "node created, attempt to publish");
  int8_t ret = attempt_publish<test_communication::msg::Empty>(
    node, topic_name, get_messages_empty());
  // if (message == "Empty") {
  //   publish<test_communication::msg::Empty>(node, message, get_messages_empty());
  // } else if (message == "Primitives") {
  //   publish<test_communication::msg::Primitives>(node, message, get_messages_primitives());
  // } else if (message == "StaticArrayPrimitives") {
  //   publish<test_communication::msg::StaticArrayPrimitives>(
  //     node, message, get_messages_static_array_primitives());
  // } else if (message == "DynamicArrayPrimitives") {
  //   publish<test_communication::msg::DynamicArrayPrimitives>(
  //     node, message, get_messages_dynamic_array_primitives());
  // } else if (message == "BoundedArrayPrimitives") {
  //   publish<test_communication::msg::BoundedArrayPrimitives>(
  //     node, message, get_messages_bounded_array_primitives());
  // } else if (message == "Nested") {
  //   publish<test_communication::msg::Nested>(node, message, get_messages_nested());
  // } else if (message == "DynamicArrayNested") {
  //   publish<test_communication::msg::DynamicArrayNested>(
  //     node, message, get_messages_dynamic_array_nested());
  // } else if (message == "BoundedArrayNested") {
  //   publish<test_communication::msg::BoundedArrayNested>(
  //     node, message, get_messages_bounded_array_nested());
  // } else if (message == "StaticArrayNested") {
  //   publish<test_communication::msg::StaticArrayNested>(
  //     node, message, get_messages_static_array_nested());
  // } else if (message == "Builtins") {
  //   publish<test_communication::msg::Builtins>(node, message, get_messages_builtins());
  // } else {
  //   fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
  //   return 1;
  // }
  return ret;
}
