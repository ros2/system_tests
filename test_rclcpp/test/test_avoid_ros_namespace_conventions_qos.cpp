// Copyright 2017 Open Source Robotics Foundation, Inc.
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

// Test communciation works with the avoid_ros_namespace_conventions QoS enabled.
TEST(
  CLASSNAME(test_avoid_ros_namespace_conventions_qos, RMW_IMPLEMENTATION),
  pub_sub_works
) {
  // topic name
  std::string topic_name = "test_avoid_ros_namespace_conventions_qos";
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
  rmw_qos_profile_t custom_qos = rmw_qos_profile_default;
  custom_qos.avoid_ros_namespace_conventions = true;
  auto create_subscription_func =
    [&callback, &custom_qos](
    rclcpp::Node::SharedPtr node,
    const std::string & topic_name) -> rclcpp::Subscription<test_rclcpp::msg::UInt32>::SharedPtr
    {
      auto subscriber = node->create_subscription<test_rclcpp::msg::UInt32>(
        topic_name, callback, custom_qos);
      return subscriber;
    };
  // code to do the publish function
  auto publish_func =
    [&custom_qos](
    rclcpp::Publisher<test_rclcpp::msg::UInt32>::SharedPtr publisher,
    test_rclcpp::msg::UInt32::SharedPtr msg)
    {
      publisher->publish(msg);
    };
  // call the test template
  single_message_pub_sub_fixture<test_rclcpp::msg::UInt32>(
    topic_name, counter, create_subscription_func, publish_func, custom_qos);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
