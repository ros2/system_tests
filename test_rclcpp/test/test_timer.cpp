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

TEST(test_time, timer_fire_regularly)
{
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_timer_fire_regularly");

  size_t counter = 0;
  auto callback =
    [&counter]() -> void
    {
      ++counter;
      std::cout << "  callback() " << counter << std::endl;
    };

  rclcpp::executors::SingleThreadedExecutor executor;

  std::chrono::milliseconds period(1000);
  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), callback);

  // start condition
  ASSERT_EQ(0, counter);

  auto start = std::chrono::steady_clock::now();

  // before the first callback
  std::cout << "sleep for half interval - no callback expected" << std::endl;
  std::this_thread::sleep_for(period / 2);
  ASSERT_EQ(0, counter);

  // spin for several periods
  std::cout << "spin_node_some() for 4s" << std::endl;
  while (std::chrono::steady_clock::now() < start + 4.5 * period) {
    executor.spin_node_some(node);
    std::this_thread::sleep_for(period / 25);
  }

  // check number of callbacks
  std::cout << "expecting 4 callbacks" << std::endl;
  ASSERT_EQ(4, counter);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "running for " << diff.count() << " seconds" << std::endl;
}

TEST(test_time, timer_during_wait)
{
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_timer_during_wait");

  size_t counter = 0;
  auto callback =
    [&counter]() -> void
    {
      ++counter;
      std::cout << "  callback() " << counter << std::endl;
    };

  rclcpp::executors::SingleThreadedExecutor executor;

  std::chrono::milliseconds period(1000);
  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), callback);

  // start condition
  ASSERT_EQ(0, counter);

  auto start = std::chrono::steady_clock::now();

  // before the first callback
  std::cout << "sleep for half interval - no callback expected" << std::endl;
  std::this_thread::sleep_for(period / 2);
  ASSERT_EQ(0, counter);

  auto spinner =
    [&executor, &node]() -> void
    {
      std::cout << "spin() until shutdown" << std::endl;
      executor.add_node(node, false);
      executor.spin();
    };

  auto thread = std::thread(spinner);
  std::cout << "sleeping for 4 periods" << std::endl;
  std::this_thread::sleep_for(4 * period);
  std::cout << "shutdown()" << std::endl;
  rclcpp::shutdown();
  thread.join();

  // check number of callbacks
  std::cout << "expecting 4 callbacks" << std::endl;
  ASSERT_EQ(4, counter);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "running for " << diff.count() << " seconds" << std::endl;
}
