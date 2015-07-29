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

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <test_rclcpp/msg/u_int32.hpp>

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), subscription_and_spinning)
{
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>("test_subscription", 10);

  size_t counter = 0;
  auto callback =
    [&counter](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++counter;
      std::cout << "  callback() " << counter << " with message data " << msg->data << std::endl;
      ASSERT_EQ(counter, msg->data);
    };

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", 10, callback);

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  // start condition
  ASSERT_EQ(0, counter);

  // nothing should be pending here
  std::cout << "spin_node_once(nonblocking) - no callback expected" << std::endl;
  executor.spin_node_once(node, true);
  ASSERT_EQ(0, counter);
  std::cout << "spin_node_some() - no callback expected" << std::endl;
  executor.spin_node_some(node);
  ASSERT_EQ(0, counter);

  msg->data = 1;
  publisher->publish(msg);
  ASSERT_EQ(0, counter);

  // wait for the first callback
  std::cout << "spin_node_once() - callback (1) expected" << std::endl;
  executor.spin_node_once(node);
  ASSERT_EQ(1, counter);

  // nothing should be pending here
  std::cout << "spin_node_once(nonblocking) - no callback expected" << std::endl;
  executor.spin_node_once(node, true);
  ASSERT_EQ(1, counter);
  std::cout << "spin_node_some() - no callback expected" << std::endl;
  executor.spin_node_some(node);
  ASSERT_EQ(1, counter);

  msg->data = 2;
  publisher->publish(msg);
  msg->data = 3;
  publisher->publish(msg);
  msg->data = 4;
  publisher->publish(msg);
  msg->data = 5;
  publisher->publish(msg);
  ASSERT_EQ(1, counter);

  // while four messages have been published one callback should be triggered here
  std::cout << "spin_node_once() - callback (2) expected" << std::endl;
  executor.spin_node_once(node, true);
  ASSERT_EQ(2, counter);

  // check for next pending call
  std::cout << "spin_node_once() - callback (3) expected" << std::endl;
  executor.spin_node_once(node, true);
  ASSERT_EQ(3, counter);

  // check for all remaning calls
  std::cout << "spin_node_some() - callbacks (4 and 5) expected" << std::endl;
  executor.spin_node_some(node);
  ASSERT_EQ(5, counter);
}
