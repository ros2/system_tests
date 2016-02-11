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

#include <atomic>
#include <stdexcept>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), recursive_spin_call) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = rclcpp::Node::make_shared("recursive_spin_call");
  auto timer = node->create_wall_timer(0_s, [&executor]() {
    ASSERT_THROW(executor.spin_some(), std::runtime_error);
    ASSERT_THROW(executor.spin_once(), std::runtime_error);
    ASSERT_THROW(executor.spin(), std::runtime_error);
    executor.cancel();
  });
  executor.add_node(node);
  executor.spin();
}

TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), multithreaded_spin_call) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = rclcpp::Node::make_shared("multithreaded_spin_call");
  std::mutex m;
  bool ready = false;
  std::condition_variable cv;
  std::thread t([&executor, &m, &cv, &ready]() {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [&ready] {return ready; });
    ASSERT_THROW(executor.spin_some(), std::runtime_error);
    ASSERT_THROW(executor.spin_once(), std::runtime_error);
    ASSERT_THROW(executor.spin(), std::runtime_error);
    executor.cancel();
  });
  auto timer = node->create_wall_timer(0_s, [&m, &cv, &ready]() {
    if (!ready) {
      {
        std::lock_guard<std::mutex> lock(m);
        ready = true;
      }
      cv.notify_one();
    }
  });
  executor.add_node(node);
  executor.spin();
  t.join();
}

// Try spinning 2 single-threaded executors in two separate threads.
TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), multiple_executors) {
  std::atomic_uint counter1;
  counter1 = 0;
  std::atomic_uint counter2;
  counter2 = 0;
  const uint32_t counter_goal = 20;

  // Initialize executor 1.
  rclcpp::executors::SingleThreadedExecutor executor1;
  auto callback1 = [&counter1, &counter_goal, &executor1]() {
      if (counter1 == counter_goal) {
        executor1.cancel();
        return;
      }
      ++counter1;
    };
  auto node1 = rclcpp::Node::make_shared("multiple_executors_1");
  auto timer1 = node1->create_wall_timer(1_ms, callback1);
  executor1.add_node(node1);

  // Initialize executor 2.
  rclcpp::executors::SingleThreadedExecutor executor2;

  auto callback2 = [&counter2, &counter_goal, &executor2]() {
      if (counter2 == counter_goal) {
        executor2.cancel();
        return;
      }
      ++counter2;
    };
  auto node2 = rclcpp::Node::make_shared("multiple_executors_2");
  auto timer2 = node2->create_wall_timer(1_ms, callback2);
  executor2.add_node(node2);

  auto spin_executor2 = [&executor2]() {
      executor2.spin();
    };

  // Launch both executors
  std::thread execution_thread(spin_executor2);

  executor1.spin();
  execution_thread.join();
  EXPECT_EQ(counter1.load(), counter_goal);
  EXPECT_EQ(counter2.load(), counter_goal);

  // Try to add node1 to executor2. It should throw, since node1 was already added to executor1.
  ASSERT_THROW(executor2.add_node(node1), std::runtime_error);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
