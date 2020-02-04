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
#include <thread>
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
  auto qos = rclcpp::QoS(rclcpp::KeepLast(messages.size()));

  auto publisher = node->create_publisher<T>(std::string("test_message_") + message_type, qos);

  rclcpp::WallRate cycle_rate(10);
  rclcpp::WallRate message_rate(100);
  size_t cycle_index = 0;
  // publish all messages up to number_of_cycles times, longer sleep between each cycle
  while (rclcpp::ok() && cycle_index < number_of_cycles) {
    size_t message_index = 0;
    // publish all messages one by one, shorter sleep between each message
    while (rclcpp::ok() && message_index < messages.size()) {
      printf("publishing message #%zu\n", message_index + 1);
      publisher->publish(*messages[message_index]);
      ++message_index;
      message_rate.sleep();
    }
    ++cycle_index;
    cycle_rate.sleep();
  }
}

template<typename T>
rclcpp::SubscriptionBase::SharedPtr subscribe(
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
          printf("received message #%zu of %zu\n", index + 1, expected_messages.size());
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

  auto qos = rclcpp::QoS(rclcpp::KeepLast(expected_messages.size()));

  auto subscriber =
    node->create_subscription<T>(std::string("test_message_") + message_type, qos, callback);
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

  rclcpp::SubscriptionBase::SharedPtr subscriber;
  std::vector<bool> received_messages;  // collect flags about received messages

  auto messages_empty = get_messages_empty();
  auto messages_basic_types = get_messages_basic_types();
  auto messages_arrays = get_messages_arrays();
  auto messages_bounded_sequences = get_messages_bounded_sequences();
  auto messages_unbounded_sequences = get_messages_unbounded_sequences();
  auto messages_multi_nested = get_messages_multi_nested();
  auto messages_nested = get_messages_nested();
  auto messages_builtins = get_messages_builtins();
  auto messages_constants = get_messages_constants();
  auto messages_defaults = get_messages_defaults();
  auto messages_strings = get_messages_strings();
  auto messages_wstrings = get_messages_wstrings();

  std::thread spin_thread([node]() {
      rclcpp::spin(node);
    });

  if (message == "Empty") {
    subscriber = subscribe<test_msgs::msg::Empty>(
      node, message, messages_empty, received_messages);
    publish<test_msgs::msg::Empty>(node, message, messages_empty);
  } else if (message == "BasicTypes") {
    subscriber = subscribe<test_msgs::msg::BasicTypes>(
      node, message, messages_basic_types, received_messages);
    publish<test_msgs::msg::BasicTypes>(node, message, messages_basic_types);
  } else if (message == "Arrays") {
    subscriber = subscribe<test_msgs::msg::Arrays>(
      node, message, messages_arrays, received_messages);
    publish<test_msgs::msg::Arrays>(node, message, messages_arrays);
  } else if (message == "UnboundedSequences") {
    subscriber = subscribe<test_msgs::msg::UnboundedSequences>(
      node, message, messages_unbounded_sequences, received_messages);
    publish<test_msgs::msg::UnboundedSequences>(
      node, message, messages_unbounded_sequences);
  } else if (message == "BoundedSequences") {
    subscriber = subscribe<test_msgs::msg::BoundedSequences>(
      node, message, messages_bounded_sequences, received_messages);
    publish<test_msgs::msg::BoundedSequences>(
      node, message, messages_bounded_sequences);
  } else if (message == "MultiNested") {
    subscriber = subscribe<test_msgs::msg::MultiNested>(
      node, message, messages_multi_nested, received_messages);
    publish<test_msgs::msg::MultiNested>(node, message, messages_multi_nested);
  } else if (message == "Nested") {
    subscriber = subscribe<test_msgs::msg::Nested>(
      node, message, messages_nested, received_messages);
    publish<test_msgs::msg::Nested>(node, message, messages_nested);
  } else if (message == "Builtins") {
    subscriber = subscribe<test_msgs::msg::Builtins>(
      node, message, messages_builtins, received_messages);
    publish<test_msgs::msg::Builtins>(node, message, messages_builtins);
  } else if (message == "Constants") {
    subscriber = subscribe<test_msgs::msg::Constants>(
      node, message, messages_constants, received_messages);
    publish<test_msgs::msg::Constants>(node, message, messages_constants);
  } else if (message == "Defaults") {
    subscriber = subscribe<test_msgs::msg::Defaults>(
      node, message, messages_defaults, received_messages);
    publish<test_msgs::msg::Defaults>(node, message, messages_defaults);
  } else if (message == "Strings") {
    subscriber = subscribe<test_msgs::msg::Strings>(
      node, message, messages_strings, received_messages);
    publish<test_msgs::msg::Strings>(node, message, messages_strings);
  } else if (message == "WStrings") {
    subscriber = subscribe<test_msgs::msg::WStrings>(
      node, message, messages_wstrings, received_messages);
    publish<test_msgs::msg::WStrings>(node, message, messages_wstrings);
  } else {
    fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
    return 1;
  }

  spin_thread.join();

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("published and subscribed for %f seconds\n", diff.count());

  for (auto received : received_messages) {
    if (!received) {
      return 1;
    }
  }

  return 0;
}
