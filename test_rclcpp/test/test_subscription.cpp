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

#include "test_rclcpp/msg/u_int32.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

static const std::chrono::milliseconds sleep_per_loop(10);
static const int max_loops = 200;

TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), subscription_and_spinning) {
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_subscription");

  rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
  custom_qos_profile.depth = 10;

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_subscription", custom_qos_profile);

  uint32_t counter = 0;
  auto callback =
    [&counter](const test_rclcpp::msg::UInt32::SharedPtr msg) -> void
    {
      ++counter;
      printf("  callback() %4u with message data %u\n", counter, msg->data);
      ASSERT_EQ(counter, msg->data);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  {
    auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
      "test_subscription", callback, custom_qos_profile);

    // start condition
    ASSERT_EQ(0, counter);

    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    ASSERT_EQ(0, counter);
    printf("spin_node_some() - no callback expected\n");
    executor.spin_node_some(node);
    ASSERT_EQ(0, counter);

    msg->data = 1;
    publisher->publish(msg);
    ASSERT_EQ(0, counter);

    // wait for the first callback
    printf("spin_node_once() - callback (1) expected\n");
    executor.spin_node_once(node);
    ASSERT_EQ(1, counter);

    // nothing should be pending here
    printf("spin_node_once(nonblocking) - no callback expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    ASSERT_EQ(1, counter);
    printf("spin_node_some() - no callback expected\n");
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
    printf("spin_node_once(nonblocking) - callback (2) expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    if (counter == 1) {
      // give the executor thread time to process the event
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      printf("spin_node_once(nonblocking) - callback (2) expected - trying again\n");
      executor.spin_node_once(node, std::chrono::milliseconds(0));
    }
    ASSERT_EQ(2, counter);

    // check for next pending call
    printf("spin_node_once(nonblocking) - callback (3) expected\n");
    executor.spin_node_once(node, std::chrono::milliseconds(0));
    if (counter == 2) {
      // give the executor thread time to process the event
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      printf("spin_node_once(nonblocking) - callback (3) expected - trying again\n");
      executor.spin_node_once(node, std::chrono::milliseconds(0));
    }
    ASSERT_EQ(3, counter);

    // check for all remaning calls
    printf("spin_node_some() - callbacks (4 and 5) expected\n");
    executor.spin_node_some(node);
    if (counter == 3 || counter == 4) {
      // give the executor thread time to process the event
      std::this_thread::sleep_for(std::chrono::milliseconds(25));
      printf("spin_node_some() - callback (%s) expected - trying again\n",
        counter == 3 ? "4 and 5" : "5");
      executor.spin_node_once(node, std::chrono::milliseconds(0));
    }
    ASSERT_EQ(5, counter);
  }
  // the subscriber goes out of scope and should be not receive any callbacks anymore

  // wait a moment for everything to initialize
  // TODO(gerkey): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);

  msg->data = 6;
  publisher->publish(msg);

  std::this_thread::sleep_for(std::chrono::milliseconds(25));

  // check that no further callbacks have been invoked
  printf("spin_node_some() - no callbacks expected\n");
  executor.spin_node_some(node);
  ASSERT_EQ(5, counter);
}

// Shortened version of the test for the ConstSharedPtr callback signature
TEST(CLASSNAME(test_subscription, RMW_IMPLEMENTATION), subscription_shared_ptr_const) {
  auto node = rclcpp::Node::make_shared("test_subscription");

  auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
    "test_subscription", rmw_qos_profile_default);

  uint32_t counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::ConstSharedPtr msg) -> void
    {
      ++counter;
      printf("  callback() %4u with message data %u\n", counter, msg->data);
      ASSERT_EQ(counter, msg->data);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", callback, rmw_qos_profile_default);

  // wait a moment for everything to initialize
  // TODO(gerkey): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);

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
  int loop = 0;
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
  uint32_t counter;
  void callback(test_rclcpp::msg::UInt32::ConstSharedPtr msg)
  {
    ++counter;
    ASSERT_EQ(counter, msg->data);
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
  // TODO(gerkey): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);

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
  int loop = 0;
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
  // TODO(gerkey): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);

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
  int loop = 0;
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

  uint32_t counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::ConstSharedPtr msg,
      const rmw_message_info_t & info) -> void
    {
      ++counter;
      printf("  callback() %4u with message data %u\n", counter, msg->data);
      ASSERT_EQ(counter, msg->data);
      ASSERT_FALSE(info.from_intra_process);
    };

  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;
  rclcpp::executors::SingleThreadedExecutor executor;

  auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
    "test_subscription", callback, rmw_qos_profile_default);

  // wait a moment for everything to initialize
  // TODO(gerkey): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);

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
  int loop = 0;
  while ((counter != 1) && (loop++ < max_loops)) {
    printf("callback not called, sleeping and trying again\n");
    std::this_thread::sleep_for(sleep_per_loop);
    executor.spin_node_some(node);
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
