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

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), spin_until_future_complete) {
  auto node = rclcpp::Node::make_shared("test_spin");

  // Construct a fake future to wait on
  std::promise<bool> promise;
  std::shared_future<bool> future(promise.get_future());

  // Make a timer to complete the promise in the future
  int i = 0;
  auto callback = [&promise, &i]() {
      if (i > 0) {
        promise.set_value(true);
      }
      ++i;
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(25), callback);

  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future),
    rclcpp::executors::FutureReturnCode::SUCCESS);
  EXPECT_EQ(future.get(), true);
}

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), spin_until_future_complete_timeout) {
  auto node = rclcpp::Node::make_shared("test_spin");

  // Construct a fake future to wait on
  std::promise<bool> promise;
  std::shared_future<bool> future(promise.get_future());

  // Make a timer to complete the promise in the future
  int i = 0;
  auto callback = [&promise, &i]() {
      if (i > 0) {
        promise.set_value(true);
      }
      ++i;
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(50), callback);

  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(25)),
    rclcpp::executors::FutureReturnCode::TIMEOUT);

  // If we wait a little longer, we should complete the future
  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(50)),
    rclcpp::executors::FutureReturnCode::SUCCESS);

  EXPECT_EQ(future.get(), true);
}

TEST(CLASSNAME(test_spin, RMW_IMPLEMENTATION), spin_until_future_complete_interrupted) {
  auto node = rclcpp::Node::make_shared("test_spin");

  // Construct a fake future to wait on
  std::promise<bool> promise;
  std::shared_future<bool> future(promise.get_future());

  // Make a timer to complete the promise in the future
  int i = 0;
  auto callback = [&promise, &i]() {
      if (i > 0) {
        promise.set_value(true);
      }
      ++i;
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(50), callback);

  // Create a timer that will shut down rclcpp before
  auto shutdown_callback = []() {
      rclcpp::utilities::shutdown();
    };
  auto shutdown_timer = node->create_wall_timer(std::chrono::milliseconds(25), shutdown_callback);

  ASSERT_EQ(rclcpp::spin_until_future_complete(node, future, std::chrono::milliseconds(50)),
    rclcpp::executors::FutureReturnCode::INTERRUPTED);
}

int main(int argc, char ** argv)
{
  // NOTE: use custom main to ensure that rclcpp::init is called only once
  rclcpp::init(0, nullptr);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
