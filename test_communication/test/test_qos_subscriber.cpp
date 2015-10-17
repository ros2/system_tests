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

#include <rclcpp/rclcpp.hpp>

#include "message_fixtures.hpp"


template<typename T>
rclcpp::subscription::SubscriptionBase::SharedPtr subscribe(
  rclcpp::Node::SharedPtr node,
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
  custom_qos_profile.durability = RMW_QOS_POLICY_TRANSIENT_LOCAL_DURABILITY;

  auto subscriber = node->create_subscription<T>(
    std::string("test_qos_message_primitives"), custom_qos_profile, callback);
  return subscriber;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto start = std::chrono::steady_clock::now();

  auto node = rclcpp::Node::make_shared(std::string("test_qos_subscriber_primitives"));

  rclcpp::subscription::SubscriptionBase::SharedPtr subscriber;
  std::vector<bool> received_messages;  // collect flags about received messages

  auto messages = get_messages_primitives();

  std::cout << "Wait for 20 seconds before subscribing" << std::endl;

  // Wait for a while before subscribing
  std::chrono::seconds wait_time(10);

  std::this_thread::sleep_for(wait_time);

  std::cout << "Create subscriber" << std::endl;

  subscriber = subscribe<test_communication::msg::Primitives>(
    node, messages, received_messages);

  rclcpp::spin(node);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "subscribed for " << diff.count() << " seconds" << std::endl;

  return 0;
}
