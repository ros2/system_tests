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
#include <future>
#include <iostream>
#include <memory>
#include <string>

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

static const std::chrono::milliseconds sleep_per_loop(10);
static const size_t max_loops = 200;


template<typename DurationT>
void wait_for_future(
  rclcpp::executor::Executor & executor,
  std::shared_future<void> & future,
  DurationT & timeout)
{
  using rclcpp::executor::FutureReturnCode;
  rclcpp::executor::FutureReturnCode future_ret;
  auto start_time = std::chrono::steady_clock::now();
  future_ret = executor.spin_until_future_complete(future, timeout);
  auto elapsed_time = std::chrono::steady_clock::now() - start_time;
  // *INDENT-OFF*
  EXPECT_EQ(FutureReturnCode::SUCCESS, future_ret)
    << "future failed to be set after: "
    << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_time).count()
    << " milliseconds\n";
  // *INDENT-ON*
}

TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), subscription_and_spinning) {
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_subscription");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 10;
  std::string topic = "test_subscription";

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    topic, custom_qos_profile);

  int counter = 0;
  std::promise<void> sub_called;
  std::shared_future<void> sub_called_future(sub_called.get_future());
  auto fail_after_timeout = 5_s;
  auto callback =
    [&counter, &sub_called](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++counter;
      printf("  callback() %d with message data %u\n", counter, msg->data);
      ASSERT_GE(counter, 0);
      ASSERT_EQ(static_cast<unsigned int>(counter), msg->data);
      sub_called.set_value();
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  {
    auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
      topic, callback, custom_qos_profile);

    // wait for discovery and the subscriber to connect
    test_rclcpp::busy_wait_for_subscriber(node, topic);

    // start condition
    ASSERT_EQ(0, counter);

    // nothing should be pending here
    printf("spin_once(nonblocking) - no callback expected\n");
    executor.spin_once(0_s);
    ASSERT_EQ(0, counter);
    printf("spin_some() - no callback expected\n");
    executor.spin_some();
    ASSERT_EQ(0, counter);

    msg->data = 1;
    publisher->publish(msg);
    ASSERT_EQ(0, counter);

    // spin until the subscription is called or a timeout occurs
    printf("spin_until_future_complete(sub_called_future) - callback (1) expected\n");
    wait_for_future(executor, sub_called_future, fail_after_timeout);
    ASSERT_EQ(1, counter);

    // no additional calls to the subscription should be pending here
    printf("spin_once(nonblocking) - no callback expected\n");
    executor.spin_once(0_s);
    ASSERT_EQ(1, counter);
    printf("spin_some() - no callback expected\n");
    executor.spin_some();
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
    printf("spin_until_future_complete(short timeout) - callback (2) expected\n");
    sub_called = std::promise<void>();
    sub_called_future = sub_called.get_future();
    wait_for_future(executor, sub_called_future, 10_ms);
    ASSERT_EQ(2, counter);

    // check for next pending call
    printf("spin_until_future_complete(short timeout) - callback (3) expected\n");
    sub_called = std::promise<void>();
    sub_called_future = sub_called.get_future();
    wait_for_future(executor, sub_called_future, 10_ms);
    ASSERT_EQ(3, counter);

    // check for next pending call
    printf("spin_until_future_complete(short timeout) - callback (4) expected\n");
    sub_called = std::promise<void>();
    sub_called_future = sub_called.get_future();
    wait_for_future(executor, sub_called_future, 10_ms);
    ASSERT_EQ(4, counter);

    // check for last pending call (blocking)
    printf("spin_until_future_complete() - callback (5) expected\n");
    sub_called = std::promise<void>();
    sub_called_future = sub_called.get_future();
    wait_for_future(executor, sub_called_future, fail_after_timeout);
    ASSERT_EQ(5, counter);
  }
  // the subscriber goes out of scope and should be not receive any callbacks anymore

  msg->data = 6;
  publisher->publish(msg);

  // check that no further callbacks have been invoked
  printf("spin_until_future_complete(short timeout) - no callbacks expected\n");
  sub_called = std::promise<void>();
  sub_called_future = sub_called.get_future();
  using rclcpp::executor::FutureReturnCode;
  rclcpp::executor::FutureReturnCode future_ret =
    executor.spin_until_future_complete(sub_called_future, 100_ms);
  EXPECT_EQ(FutureReturnCode::TIMEOUT, future_ret);
  ASSERT_EQ(5, counter);
}

// Shortened version of the test for the ConstSharedPtr callback signature
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), subscription_shared_ptr_const) {
  auto node = rclcpp::Node::make_shared("test_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_subscription", rmw_qos_profile_default);

  int counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::ConstSharedPtr msg) -> void
    {
      ++counter;
      printf("  callback() %d with message data %u\n", counter, msg->data);
      ASSERT_GE(counter, 0);
      ASSERT_EQ(static_cast<unsigned int>(counter), msg->data);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", callback, rmw_qos_profile_default);

  // wait a moment for everything to initialize
  test_rclcpp::busy_wait_for_subscriber(node, "test_subscription");

  // start condition
  ASSERT_EQ(0, counter);

  // nothing should be pending here
  executor.spin_node_some(node);
  ASSERT_EQ(0, counter);

  msg->data = 1;
  // Create a ConstSharedPtr message to publish
  test_rclcpp::msg::UInt32::ConstSharedPtr const_msg(msg);
  publisher->publish(const_msg);
  ASSERT_EQ(0, counter);

  // wait for the first callback
  printf("spin_node_some() - callback (1) expected\n");

  executor.spin_node_some(node);
  // spin for up to 2s
  size_t loop = 0;
  while ((counter != 1) && (loop++ < max_loops)) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(sleep_per_loop);
    executor.spin_node_some(node);
  }
  ASSERT_EQ(1, counter);
}

class CallbackHolder
{
public:
  int counter;
  void callback(test_rclcpp::msg::UInt32::ConstSharedPtr msg)
  {
    ++counter;
    ASSERT_GE(counter, 0);
    ASSERT_EQ(static_cast<unsigned int>(counter), msg->data);
  }
  CallbackHolder()
  : counter(0)
  {}
};

// Shortened version of the test for the ConstSharedPtr callback signature in a method
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION),
  subscription_shared_ptr_const_method_std_function) {
  CallbackHolder cb_holder;

  auto node = rclcpp::Node::make_shared("test_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_subscription", rmw_qos_profile_default);

  std::function<void(test_rclcpp::msg::UInt32::ConstSharedPtr)> cb_std_function = std::bind(
    &CallbackHolder::callback, &cb_holder, std::placeholders::_1);

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", cb_std_function, rmw_qos_profile_default);

  // wait a moment for everything to initialize
  test_rclcpp::busy_wait_for_subscriber(node, "test_subscription");

  // start condition
  ASSERT_EQ(0, cb_holder.counter);

  // nothing should be pending here
  executor.spin_node_some(node);
  ASSERT_EQ(0, cb_holder.counter);

  msg->data = 1;
  // Create a ConstSharedPtr message to publish
  test_rclcpp::msg::UInt32::ConstSharedPtr const_msg(msg);
  publisher->publish(const_msg);
  ASSERT_EQ(0, cb_holder.counter);

  // wait for the first callback
  printf("spin_node_some() - callback (1) expected\n");

  executor.spin_node_some(node);
  // spin for up to 2s
  size_t loop = 0;
  while ((cb_holder.counter != 1) && (loop++ < max_loops)) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(sleep_per_loop);
    executor.spin_node_some(node);
  }
  ASSERT_EQ(1, cb_holder.counter);
}

// Shortened version of the test for the ConstSharedPtr callback signature in a method
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION),
  subscription_shared_ptr_const_method_direct) {
  CallbackHolder cb_holder;

  auto node = rclcpp::Node::make_shared("test_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_subscription", rmw_qos_profile_default);

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription",
    std::bind(&CallbackHolder::callback, &cb_holder, std::placeholders::_1),
    rmw_qos_profile_default);

  // wait a moment for everything to initialize
  test_rclcpp::busy_wait_for_subscriber(node, "test_subscription");

  // start condition
  ASSERT_EQ(0, cb_holder.counter);

  // nothing should be pending here
  executor.spin_node_some(node);
  ASSERT_EQ(0, cb_holder.counter);

  msg->data = 1;
  // Create a ConstSharedPtr message to publish
  test_rclcpp::msg::UInt32::ConstSharedPtr const_msg(msg);
  publisher->publish(const_msg);
  ASSERT_EQ(0, cb_holder.counter);

  // wait for the first callback
  printf("spin_node_some() - callback (1) expected\n");

  executor.spin_node_some(node);
  // spin for up to 2s
  size_t loop = 0;
  while ((cb_holder.counter != 1) && (loop++ < max_loops)) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(sleep_per_loop));
    executor.spin_node_some(node);
  }
  ASSERT_EQ(1, cb_holder.counter);
}

// Shortened version of the test for the ConstSharedPtr with info callback signature
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), subscription_shared_ptr_const_with_info) {
  auto node = rclcpp::Node::make_shared("test_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_subscription", rmw_qos_profile_default);

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

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", callback, rmw_qos_profile_default);

  // wait a moment for the subscriber to register
  test_rclcpp::busy_wait_for_subscriber(node, "test_subscription");

  // start condition
  ASSERT_EQ(0, counter);

  // nothing should be pending here
  executor.spin_node_some(node);
  ASSERT_EQ(0, counter);

  msg->data = 1;
  publisher->publish(msg);
  ASSERT_EQ(0, counter);

  // wait for the first callback
  printf("spin_node_some() - callback (1) expected\n");

  executor.spin_node_some(node);
  // spin for up to 2s
  size_t loop = 0;
  while ((counter != 1) && (loop++ < max_loops)) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(sleep_per_loop);
    executor.spin_node_some(node);
  }
  ASSERT_EQ(1, counter);
}

// Shortened version of the test for subscribing after spinning has started.
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), spin_before_subscription) {
  auto node = rclcpp::Node::make_shared("spin_before_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "spin_before_subscription", rmw_qos_profile_default);

  int counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::ConstSharedPtr msg) -> void
    {
      ++counter;
      printf("  callback() %d with message data %u\n", counter, msg->data);
      ASSERT_GE(counter, 0);
      ASSERT_EQ(static_cast<unsigned int>(counter), msg->data);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  executor.spin_some();

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "spin_before_subscription", callback, rmw_qos_profile_default);

  // start condition
  ASSERT_EQ(0, counter);

  test_rclcpp::busy_wait_for_subscriber(node, "spin_before_subscription");

  msg->data = 1;
  // Create a ConstSharedPtr message to publish
  test_rclcpp::msg::UInt32::ConstSharedPtr const_msg(msg);
  publisher->publish(const_msg);

  // wait for the first callback
  printf("callback (1) expected\n");

  size_t loop = 0;
  executor.spin_some();
  while ((counter != 1) && (loop++ < max_loops)) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(sleep_per_loop);
    executor.spin_some();
  }

  ASSERT_EQ(1, counter);
}

// Test of the queue size create_subscription signature.
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), create_subscription_with_queue_size) {
  auto node = rclcpp::Node::make_shared("test_subscription");

  // *INDENT-OFF*
  auto callback = [](test_rclcpp::msg::UInt32::ConstSharedPtr) -> void {};
  // *INDENT-ON*

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", 10, callback);
}
