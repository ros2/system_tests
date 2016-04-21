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

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/utils.hpp"
#include "test_rclcpp/msg/u_int32.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

// Short test for the const reference publish signature.
TEST(CLASSNAME(test_publisher, RMW_IMPLEMENTATION), publish_with_const_reference) {
  rclcpp::init(0, nullptr);
  auto node = rclcpp::Node::make_shared("test_publisher");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>("test_publisher", 10);

  int counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::ConstSharedPtr msg,
      const rmw_message_info_t & info) -> void
    {
      ++counter;
      printf("  callback() %d with message data %u\n", counter, msg->data);
      ASSERT_GE(counter, 0);
      ASSERT_EQ(static_cast<unsigned int>(counter), msg->data);
      ASSERT_FALSE(info.from_intra_process);
    };

  test_rclcpp::msg::UInt32 msg;
  msg.data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_publisher", callback, rmw_qos_profile_default);

  // start condition
  ASSERT_EQ(0, counter);

  // nothing should be pending here
  executor.spin_node_some(node);
  ASSERT_EQ(0, counter);
  test_rclcpp::busy_wait_for_subscriber(node, "test_publisher");

  msg.data = 1;
  publisher->publish(msg);
  ASSERT_EQ(0, counter);

  // wait for the first callback
  printf("spin_node_some() - callback (1) expected\n");

  executor.spin_node_some(node);
  // spin up to 4 times with a 25 ms wait in between
  for (uint32_t i = 0; i < 4 && counter == 0; ++i) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
    executor.spin_node_some(node);
  }
  ASSERT_EQ(1, counter);
}
