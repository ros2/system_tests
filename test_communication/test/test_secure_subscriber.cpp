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
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "message_fixtures.hpp"


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr attempt_subscribe(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  std::vector<typename T::SharedPtr> & expected_messages,
  std::vector<bool> & received_messages)
{
  received_messages.assign(expected_messages.size(), false);

  auto callback =
    [&expected_messages, &received_messages](const typename T::SharedPtr received_message) -> void
    {
      // find received message in vector of expected messages
      auto received = received_messages.begin();
      bool known_message = false;
      size_t index = 0;
      for (auto expected_message : expected_messages) {
        if (*received_message == *expected_message) {
          *received = true;
          std::cout << "received message #" << (index + 1) << " of " <<
            expected_messages.size() << std::endl;
          known_message = true;
          break;
        }
        ++received;
        ++index;
      }
      if (!known_message) {
        fprintf(stderr, "received message does not match any expected message\n");
        rclcpp::shutdown();
        throw std::runtime_error("received message does not match any expected message");
      }

      // shutdown node when all expected messages have been received
      for (auto received_msg : received_messages) {
        if (!received_msg) {
          return;
        }
      }
      rclcpp::shutdown();
    };

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = expected_messages.size();

  auto subscriber = node->create_subscription<T>(
    topic_name, callback, custom_qos_profile);
  return subscriber;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 5) {
    fprintf(
      stderr,
      "Wrong number of arguments,\n"
      "pass a node name, a topic name and a should_throw boolean\n");
    return 1;
  }
  std::string node_name = argv[1];
  std::string topic_name = argv[2];
  bool should_throw = ((0 == strcmp(argv[3], "false")) || (0 == strcmp(argv[3], "0"))) ? false : true;
  bool should_timeout = ((0 == strcmp(argv[4], "false")) || (0 == strcmp(argv[3], "0"))) ? false : true;
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
  fprintf(stderr, "node created, attempt to subscribe");
  std::vector<bool> received_messages;  // collect flags about received messages
  rclcpp::subscription::SubscriptionBase::SharedPtr subscriber;
  // TODO(mikaelarguedas) test access control passing another should_throw
  auto messages_empty = get_messages_empty();
  subscriber = attempt_subscribe<test_communication::msg::Empty>(
    node, topic_name, messages_empty, received_messages);
  if(should_throw) {
    if (nullptr == subscriber) {
      return 0;
    } else {
      return 1;
    }
  }
  rclcpp::spin(node);

  // TODO(mikaelarguedas) use should_timeout
  (void)should_timeout;
  // auto start = std::chrono::steady_clock::now();

  // std::string message = argv[1];
  // auto node = rclcpp::Node::make_shared(std::string("test_subscriber_") + message);

  // auto messages_empty = get_messages_empty();
  // auto messages_primitives = get_messages_primitives();
  // auto messages_static_array_primitives = get_messages_static_array_primitives();
  // auto messages_dynamic_array_primitives = get_messages_dynamic_array_primitives();
  // auto messages_bounded_array_primitives = get_messages_bounded_array_primitives();
  // auto messages_nested = get_messages_nested();
  // auto messages_dynamic_array_nested = get_messages_dynamic_array_nested();
  // auto messages_bounded_array_nested = get_messages_bounded_array_nested();
  // auto messages_static_array_nested = get_messages_static_array_nested();
  // auto messages_builtins = get_messages_builtins();

  // rclcpp::subscription::SubscriptionBase::SharedPtr subscriber;
  // std::vector<bool> received_messages;  // collect flags about received messages
  // if (message == "Empty") {
  //   subscriber = subscribe<test_communication::msg::Empty>(
  //     node, message, messages_empty, received_messages);
  // } else if (message == "Primitives") {
  //   subscriber = subscribe<test_communication::msg::Primitives>(
  //     node, message, messages_primitives, received_messages);
  // } else if (message == "StaticArrayPrimitives") {
  //   subscriber = subscribe<test_communication::msg::StaticArrayPrimitives>(
  //     node, message, messages_static_array_primitives, received_messages);
  // } else if (message == "DynamicArrayPrimitives") {
  //   subscriber = subscribe<test_communication::msg::DynamicArrayPrimitives>(
  //     node, message, messages_dynamic_array_primitives, received_messages);
  // } else if (message == "BoundedArrayPrimitives") {
  //   subscriber = subscribe<test_communication::msg::BoundedArrayPrimitives>(
  //     node, message, messages_bounded_array_primitives, received_messages);
  // } else if (message == "Nested") {
  //   subscriber = subscribe<test_communication::msg::Nested>(
  //     node, message, messages_nested, received_messages);
  // } else if (message == "DynamicArrayNested") {
  //   subscriber = subscribe<test_communication::msg::DynamicArrayNested>(
  //     node, message, messages_dynamic_array_nested, received_messages);
  // } else if (message == "BoundedArrayNested") {
  //   subscriber = subscribe<test_communication::msg::BoundedArrayNested>(
  //     node, message, messages_bounded_array_nested, received_messages);
  // } else if (message == "StaticArrayNested") {
  //   subscriber = subscribe<test_communication::msg::StaticArrayNested>(
  //     node, message, messages_static_array_nested, received_messages);
  // } else if (message == "Builtins") {
  //   subscriber = subscribe<test_communication::msg::Builtins>(
  //     node, message, messages_builtins, received_messages);
  // } else {
  //   fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
  //   return 1;
  // }
  // rclcpp::spin(node);

  // auto end = std::chrono::steady_clock::now();
  // std::chrono::duration<float> diff = (end - start);
  // std::cout << "subscribed for " << diff.count() << " seconds" << std::endl;

  return 0;
}
