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

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

TEST(CLASSNAME(test_time, RMW_IMPLEMENTATION), timer_fire_regularly) {
  auto node = rclcpp::Node::make_shared("test_timer_fire_regularly");

  unsigned long counter = 0;
  auto callback =
    [&counter]() -> void
    {
      ++counter;
      printf("  callback() %lu\n", counter);
    };

  rclcpp::executors::SingleThreadedExecutor executor;

  // start condition
  ASSERT_EQ(0, counter);

  auto start = std::chrono::steady_clock::now();

  std::chrono::milliseconds period(100);

  {
    auto timer = node->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period), callback);

    // before the first callback
    printf("sleep for half interval - no callback expected\n");
    std::this_thread::sleep_for(period / 2);
    ASSERT_EQ(0, counter);

    // spin for several periods
    printf("spin_node_some() for some time\n");
    while (std::chrono::steady_clock::now() < start + 4.5 * period) {
      executor.spin_node_some(node);
      std::this_thread::sleep_for(period / 10);
    }

    // check number of callbacks
    printf("expecting 4 callbacks\n");
    ASSERT_EQ(4, counter);
  }
  // the timer goes out of scope and should be not receive any callbacks anymore

  std::this_thread::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(1.5 * period));

  // check that no further callbacks have been invoked
  printf("spin_node_some() - no callbacks expected\n");
  executor.spin_node_some(node);
  ASSERT_EQ(4, counter);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("running for %.3f seconds\n", diff.count());
}

TEST(CLASSNAME(test_time, RMW_IMPLEMENTATION), timer_during_wait) {
  auto node = rclcpp::Node::make_shared("test_timer_during_wait");

  unsigned long counter = 0;
  auto callback =
    [&counter]() -> void
    {
      ++counter;
      printf("  callback() %lu", counter);
    };

  rclcpp::executors::SingleThreadedExecutor executor;

  std::chrono::milliseconds period(100);
  auto timer = node->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), callback);

  // start condition
  ASSERT_EQ(0, counter);

  auto start = std::chrono::steady_clock::now();

  // before the first callback
  printf("sleep for half interval - no callback expected\n");
  std::this_thread::sleep_for(period / 2);
  ASSERT_EQ(0, counter);

  auto spinner =
    [&executor, &node]() -> void
    {
      printf("spin() until shutdown\n");
      executor.add_node(node, false);
      executor.spin();
    };

  auto thread = std::thread(spinner);
  printf("sleeping for 4 periods\n");
  std::this_thread::sleep_for(4 * period);
  printf("shutdown()\n");
  rclcpp::shutdown();
  thread.join();

  // check number of callbacks
  printf("expecting 4 callbacks\n");
  ASSERT_EQ(4, counter);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("running for %.3f seconds\n", diff.count());
}

int main(int argc, char ** argv)
{
  // NOTE: use custom main to ensure that rclcpp::init is called only once
  rclcpp::init(0, nullptr);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
