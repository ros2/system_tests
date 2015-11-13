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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
