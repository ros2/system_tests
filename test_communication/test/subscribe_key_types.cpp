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

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/keyed_string.hpp"
#include "test_msgs/msg/non_keyed_with_nested_key.hpp"

#include "subscribe_helper.hpp"
#include "subscribe_key_types.hpp"

rclcpp::SubscriptionBase::SharedPtr subscribe_keyed_string(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<test_msgs::msg::KeyedString::SharedPtr> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::KeyedString>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_non_keyed_with_nested_key(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<test_msgs::msg::NonKeyedWithNestedKey::SharedPtr> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::NonKeyedWithNestedKey>(
    node, message_type, expected_messages, received_messages);
}

rclcpp::SubscriptionBase::SharedPtr subscribe_complex_nested_key(
  rclcpp::Node::SharedPtr node,
  const std::string & message_type,
  const std::vector<test_msgs::msg::ComplexNestedKey::SharedPtr> & expected_messages,
  std::vector<bool> & received_messages)
{
  return subscribe<test_msgs::msg::ComplexNestedKey>(
    node, message_type, expected_messages, received_messages);
}
