// Copyright 2015-2017 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "message_fixtures.hpp"

using namespace std::chrono_literals;

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

template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr attempt_subscribe(
  rclcpp::Node::SharedPtr node,
  const std::string & topic_name,
  bool & sub_callback_called,
  rclcpp::executors::SingleThreadedExecutor & exec)
{
  auto subscription_callback =
    [&sub_callback_called, &exec](const typename T::SharedPtr) -> void
    {
      fprintf(stderr, "***SUB_CALLBACK***");
      sub_callback_called = true;
      exec.cancel();
    };

  auto subscriber = node->create_subscription<T>(
    topic_name, subscription_callback, rmw_qos_profile_default);
  return subscriber;
}

rclcpp::TimerBase::SharedPtr create_timer(
  rclcpp::Node::SharedPtr node,
  bool & timer_callback_called,
  rclcpp::executors::SingleThreadedExecutor & exec)
{
  auto timer_callback =
    [&timer_callback_called, &exec]() -> void
    {
      fprintf(stderr, "***TIMER_CALLBACK***");
      timer_callback_called = true;
      exec.cancel();
    };
  auto timer = node->create_wall_timer(8s, timer_callback, nullptr);

  return timer;
}


int main(int argc, char ** argv)
{
  if (argc != 3) {
    fprintf(
      stderr,
      "Wrong number of arguments,\n"
      "pass a node name, a topic name and a should_timeout boolean\n");
    return 1;
  }
  std::string message = argv[1];
  std::string node_name = "subscriber";
  std::string topic_name = "chatter";
  bool should_timeout =
    ((0 == strcmp(argv[2], "false")) || (0 == strcmp(argv[2], "0"))) ? false : true;

  rclcpp::init(argc, argv);
  std::shared_ptr<rclcpp::node::Node> node = nullptr;
  try {
    node = rclcpp::Node::make_shared(node_name);
  } catch (std::runtime_error &) {
    fprintf(stderr, "should not have thrown!");
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::subscription::SubscriptionBase::SharedPtr subscriber;

  if (!should_timeout) {
    std::vector<bool> received_messages;  // collect flags about received messages
    auto messages_empty = get_messages_empty();
    auto messages_primitives = get_messages_primitives();
    auto messages_static_array_primitives = get_messages_static_array_primitives();
    auto messages_dynamic_array_primitives = get_messages_dynamic_array_primitives();
    auto messages_bounded_array_primitives = get_messages_bounded_array_primitives();
    auto messages_nested = get_messages_nested();
    auto messages_dynamic_array_nested = get_messages_dynamic_array_nested();
    auto messages_bounded_array_nested = get_messages_bounded_array_nested();
    auto messages_static_array_nested = get_messages_static_array_nested();
    auto messages_builtins = get_messages_builtins();
    if (message == "Empty") {
      subscriber = attempt_subscribe<test_communication::msg::Empty>(
        node, topic_name, messages_empty, received_messages);
    } else if (message == "Primitives") {
      subscriber = attempt_subscribe<test_communication::msg::Primitives>(
        node, topic_name, messages_primitives, received_messages);
    } else if (message == "StaticArrayPrimitives") {
      subscriber = attempt_subscribe<test_communication::msg::StaticArrayPrimitives>(
        node, topic_name, messages_static_array_primitives, received_messages);
    } else if (message == "DynamicArrayPrimitives") {
      subscriber = attempt_subscribe<test_communication::msg::DynamicArrayPrimitives>(
        node, topic_name, messages_dynamic_array_primitives, received_messages);
    } else if (message == "BoundedArrayPrimitives") {
      subscriber = attempt_subscribe<test_communication::msg::BoundedArrayPrimitives>(
        node, topic_name, messages_bounded_array_primitives, received_messages);
    } else if (message == "Nested") {
      subscriber = attempt_subscribe<test_communication::msg::Nested>(
        node, topic_name, messages_nested, received_messages);
    } else if (message == "DynamicArrayNested") {
      subscriber = attempt_subscribe<test_communication::msg::DynamicArrayNested>(
        node, topic_name, messages_dynamic_array_nested, received_messages);
    } else if (message == "BoundedArrayNested") {
      subscriber = attempt_subscribe<test_communication::msg::BoundedArrayNested>(
        node, topic_name, messages_bounded_array_nested, received_messages);
    } else if (message == "StaticArrayNested") {
      subscriber = attempt_subscribe<test_communication::msg::StaticArrayNested>(
        node, topic_name, messages_static_array_nested, received_messages);
    } else if (message == "Builtins") {
      subscriber = attempt_subscribe<test_communication::msg::Builtins>(
        node, topic_name, messages_builtins, received_messages);
    } else {
      fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
      rclcpp::shutdown();
      return 1;
    }

    rclcpp::spin(node);
  } else {
    bool timer_callback_called = false;
    bool sub_callback_called = false;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::executors::SingleThreadedExecutor executor;
    if (message == "Empty") {
      subscriber = attempt_subscribe<test_communication::msg::Empty>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "Primitives") {
      subscriber = attempt_subscribe<test_communication::msg::Primitives>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "StaticArrayPrimitives") {
      subscriber = attempt_subscribe<test_communication::msg::StaticArrayPrimitives>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "DynamicArrayPrimitives") {
      subscriber = attempt_subscribe<test_communication::msg::DynamicArrayPrimitives>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "BoundedArrayPrimitives") {
      subscriber = attempt_subscribe<test_communication::msg::BoundedArrayPrimitives>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "Nested") {
      subscriber = attempt_subscribe<test_communication::msg::Nested>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "DynamicArrayNested") {
      subscriber = attempt_subscribe<test_communication::msg::DynamicArrayNested>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "BoundedArrayNested") {
      subscriber = attempt_subscribe<test_communication::msg::BoundedArrayNested>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "StaticArrayNested") {
      subscriber = attempt_subscribe<test_communication::msg::StaticArrayNested>(
        node, topic_name, sub_callback_called, executor);
    } else if (message == "Builtins") {
      subscriber = attempt_subscribe<test_communication::msg::Builtins>(
        node, topic_name, sub_callback_called, executor);
    } else {
      fprintf(stderr, "Unknown message argument '%s'\n", message.c_str());
      rclcpp::shutdown();
      return 1;
    }
    timer = create_timer(node, timer_callback_called, executor);
    executor.add_node(node);
    executor.spin();
    if (!timer_callback_called || sub_callback_called) {
      rclcpp::shutdown();
      return 1;
    }
  }

  rclcpp::shutdown();
  return 0;
}
