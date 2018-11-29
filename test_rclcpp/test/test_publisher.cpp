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
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/msg/u_int32.hpp"
#include "test_rclcpp/utils.hpp"

#include "./pub_sub_fixtures.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

// Short test for the const reference publish signature.
TEST(CLASSNAME(test_publisher, RMW_IMPLEMENTATION), publish_with_const_reference) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  // topic name
  std::string topic_name = "test_publish_with_const_reference";
  // code to create the callback and subscription
  int counter = 0;
  auto callback =
    [&counter](test_rclcpp::msg::UInt32::ConstSharedPtr msg,
      const rmw_message_info_t & info) -> void
    {
      ++counter;
      printf("  callback() %d with message data %u\n", counter, msg->data);
      ASSERT_FALSE(info.from_intra_process);
    };
  auto create_subscription_func =
    [&callback](
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name) -> rclcpp::Subscription<test_rclcpp::msg::UInt32>::SharedPtr
    {
      auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
        topic_name, callback, rmw_qos_profile_default);
      return subscriber;
    };
  // code to do the publish function
  auto publish_func =
    [](
    rclcpp::Publisher<test_rclcpp::msg::UInt32>::SharedPtr publisher,
    test_rclcpp::msg::UInt32::SharedPtr msg)
    {
      publisher->publish(msg);
    };
  // call the test template
  single_message_pub_sub_fixture<test_rclcpp::msg::UInt32>(
    topic_name, counter, create_subscription_func, publish_func);
}
