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
void publish(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  std::vector<typename T::SharedPtr> messages,
  size_t number_of_cycles = 5)
{
  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = messages.size();

  auto publisher = node->create_publisher<T>(
    std::string("test_message_") + message_type, custom_qos_profile);

  rclcpp::WallRate cycle_rate(10);
  rclcpp::WallRate message_rate(40);
  size_t cycle_index = 0;
  // publish all messages up to number_of_cycles times, longer sleep between each cycle
  while (rclcpp::ok() && cycle_index < number_of_cycles) {
    size_t message_index = 0;
    // publish all messages one by one, shorter sleep between each message
    while (rclcpp::ok() && message_index < messages.size()) {
      std::cout << "publishing message #" << (message_index + 1) << std::endl;
      publisher->publish(messages[message_index]);
      ++message_index;
      message_rate.sleep();
      rclcpp::spin_some(node);
    }
    ++cycle_index;
    cycle_rate.sleep();
    rclcpp::spin_some(node);
  }
}

template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
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
      for (auto received : received_messages) {
        if (!received) {
          return;
        }
      }
      rclcpp::shutdown();
    };

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = expected_messages.size();

  auto subscriber = node->create_subscription<T>(
    std::string("test_message_") + message_type, callback, custom_qos_profile);
  return subscriber;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    fprintf(stderr, "Wrong number of arguments, pass one message type\n");
    return 1;
  }

  auto start = std::chrono::steady_clock::now();

  std::string message = argv[1];
  auto node = rclcpp::Node::make_shared(std::string("test_publisher_subscriber_") + message);

  rclcpp::subscription::SubscriptionBase::SharedPtr subscriber;
  std::vector<bool> received_messages;  // collect flags about received messages

  auto messages_empty = get_messages_empty();
  auto messages_primitives = get_messages_primitives();
  auto messages_static_array_primitives = get_messages_static_array_primitives();
  auto messages_dynamic_array_primitives = get_messages_dynamic_array_primitives();
  auto messages_nested = get_messages_nested();
  auto messages_dynamic_array_nested = get_messages_dynamic_array_nested();
  auto messages_static_array_nested = get_messages_static_array_nested();
  auto messages_builtins = get_messages_builtins();

  if (message == "Empty") {
    subscriber = subscribe<test_communication::msg::Empty>(
      node, message, messages_empty, received_messages);
    publish<test_communication::msg::Empty>(node, message, messages_empty);
  } else if (message == "Primitives") {
    subscriber = subscribe<test_communication::msg::Primitives>(
      node, message, messages_primitives, received_messages);
    publish<test_communication::msg::Primitives>(node, message, messages_primitives);
  } else if (message == "StaticArrayPrimitives") {
    subscriber = subscribe<test_communication::msg::StaticArrayPrimitives>(
      node, message, messages_static_array_primitives, received_messages);
    publish<test_communication::msg::StaticArrayPrimitives>(node, message,
      messages_static_array_primitives);
  } else if (message == "DynamicArrayPrimitives") {
    subscriber = subscribe<test_communication::msg::DynamicArrayPrimitives>(
      node, message, messages_dynamic_array_primitives, received_messages);
    publish<test_communication::msg::DynamicArrayPrimitives>(node, message,
      messages_dynamic_array_primitives);
  } else if (message == "Nested") {
    subscriber = subscribe<test_communication::msg::Nested>(
      node, message, messages_nested, received_messages);
    publish<test_communication::msg::Nested>(node, message, messages_nested);
  } else if (message == "DynamicArrayNested") {
    subscriber = subscribe<test_communication::msg::DynamicArrayNested>(
      node, message, messages_dynamic_array_nested, received_messages);
    publish<test_communication::msg::DynamicArrayNested>(node, message,
      messages_dynamic_array_nested);
  } else if (message == "StaticArrayNested") {
    subscriber = subscribe<test_communication::msg::StaticArrayNested>(
      node, message, messages_static_array_nested, received_messages);
    publish<test_communication::msg::StaticArrayNested>(node, message,
      messages_static_array_nested);
  } else if (message == "Builtins") {
    subscriber = subscribe<test_communication::msg::Builtins>(
      node, message, messages_builtins, received_messages);
    publish<test_communication::msg::Builtins>(node, message, messages_builtins);
  } else {
    fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
    return 1;
  }

  rclcpp::spin_some(node);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "published and subscribed for " << diff.count() << " seconds" << std::endl;

  for (auto received : received_messages) {
    if (!received) {
      return 1;
    }
  }

  return 0;
}
