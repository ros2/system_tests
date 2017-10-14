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

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "test_rclcpp/msg/u_int32.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

void callback(const test_rclcpp::msg::UInt32::SharedPtr /*msg*/)
{
  throw std::runtime_error("The subscriber received a message but there should be no publisher!");
}

TEST(CLASSNAME(test_timeout_subscriber, RMW_IMPLEMENTATION), timeout_subscriber) {
  rclcpp::init(0, nullptr);

  auto start = std::chrono::steady_clock::now();

  auto node = rclcpp::Node::make_shared("test_timeout_subscriber");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 10;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_message_timeout_uint32", callback, custom_qos_profile);

  rclcpp::executors::SingleThreadedExecutor executor;

  size_t num_cycles = 5;

  for (size_t i = 0; i < num_cycles; ++i) {
    auto tolerance = std::chrono::milliseconds(15);

    // ensure that the non-blocking spin does return immediately
    auto nonblocking_start = std::chrono::steady_clock::now();
    executor.spin_node_once(node, std::chrono::milliseconds::zero());
    auto nonblocking_end = std::chrono::steady_clock::now();
    auto nonblocking_diff = nonblocking_end - nonblocking_start;
    EXPECT_LT(nonblocking_diff, tolerance);

    // ensure that the blocking spin does return after the specified timeout
    auto blocking_timeout = std::chrono::milliseconds(100);
    auto blocking_start = std::chrono::steady_clock::now();
    executor.spin_node_once(node, blocking_timeout);
    auto blocking_end = std::chrono::steady_clock::now();
    auto blocking_diff = blocking_end - blocking_start;
    EXPECT_LT(blocking_diff, blocking_timeout + tolerance);
  }

  rclcpp::shutdown();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("subscribed for %f seconds\n", diff.count());
}
