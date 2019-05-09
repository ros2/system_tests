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
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/message_fixtures.hpp"


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
    node->create_subscription<T>(std::string("test/message/") + message_type, qos, callback);
  return subscriber;
}

int main(int argc, char ** argv)
{
  if (argc != 3) {
    fprintf(stderr, "Wrong number of arguments, pass one message type\n");
    return 1;
  }
  rclcpp::init(argc, argv);

  auto start = std::chrono::steady_clock::now();

  std::string message = argv[1];
  std::string namespace_ = argv[2];
  auto node = rclcpp::Node::make_shared(
    std::string("test_subscriber_") + message, namespace_);

  auto messages_empty = get_messages_empty();
  auto messages_basic_types = get_messages_basic_types();
  auto messages_arrays = get_messages_arrays();
  auto messages_unbounded_sequences = get_messages_unbounded_sequences();
  auto messages_bounded_sequences = get_messages_bounded_sequences();
  auto messages_nested = get_messages_nested();
  auto messages_multi_nested = get_messages_multi_nested();
  auto messages_builtins = get_messages_builtins();
  auto messages_constants = get_messages_constants();
  auto messages_defaults = get_messages_defaults();
  auto messages_strings = get_messages_strings();
  auto messages_wstrings = get_messages_wstrings();

  rclcpp::SubscriptionBase::SharedPtr subscriber;
  std::vector<bool> received_messages;  // collect flags about received messages
  if (message == "Empty") {
    subscriber = subscribe<test_msgs::msg::Empty>(
      node, message, messages_empty, received_messages);
  } else if (message == "BasicTypes") {
    subscriber = subscribe<test_msgs::msg::BasicTypes>(
      node, message, messages_basic_types, received_messages);
  } else if (message == "Arrays") {
    subscriber = subscribe<test_msgs::msg::Arrays>(
      node, message, messages_arrays, received_messages);
  } else if (message == "UnboundedSequences") {
    subscriber = subscribe<test_msgs::msg::UnboundedSequences>(
      node, message, messages_unbounded_sequences, received_messages);
  } else if (message == "BoundedSequences") {
    subscriber = subscribe<test_msgs::msg::BoundedSequences>(
      node, message, messages_bounded_sequences, received_messages);
  } else if (message == "MultiNested") {
    subscriber = subscribe<test_msgs::msg::MultiNested>(
      node, message, messages_multi_nested, received_messages);
  } else if (message == "Nested") {
    subscriber = subscribe<test_msgs::msg::Nested>(
      node, message, messages_nested, received_messages);
  } else if (message == "Builtins") {
    subscriber = subscribe<test_msgs::msg::Builtins>(
      node, message, messages_builtins, received_messages);
  } else if (message == "Constants") {
    subscriber = subscribe<test_msgs::msg::Constants>(
      node, message, messages_constants, received_messages);
  } else if (message == "Defaults") {
    subscriber = subscribe<test_msgs::msg::Defaults>(
      node, message, messages_defaults, received_messages);
  } else if (message == "Strings") {
    subscriber = subscribe<test_msgs::msg::Strings>(
      node, message, messages_strings, received_messages);
  } else if (message == "WStrings") {
    subscriber = subscribe<test_msgs::msg::WStrings>(
      node, message, messages_wstrings, received_messages);
  } else {
    fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
    rclcpp::shutdown();
    return 1;
  }

  try {
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "%s\n", e.what());
    rclcpp::shutdown();
    return 1;
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("subscribed for %f seconds\n", diff.count());

  rclcpp::shutdown();
  return 0;
}
