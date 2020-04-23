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

#include <gtest/gtest.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rmw/types.h"

#include "test_msgs/message_fixtures.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestMessageSerialization, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  void SetUp()
  {
    rclcpp::init(0, NULL);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

TEST_F(CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION), serialized_callback) {
  size_t counter = 0;

  auto serialized_callback =
    [&counter](const std::shared_ptr<rclcpp::SerializedMessage> serialized_msg) {
      printf("received message %zu\n", counter);
      for (auto i = 0u; i < serialized_msg->size(); ++i) {
        printf("%02x ", serialized_msg->get_rcl_serialized_message().buffer[i]);
      }
      printf("\n");

      using MessageT = test_msgs::msg::BasicTypes;
      rclcpp::Serialization<MessageT> serializer;
      MessageT basic_types_msg;
      EXPECT_NO_THROW(serializer.deserialize_message(serialized_msg.get(), &basic_types_msg));
      EXPECT_EQ(counter, basic_types_msg.uint8_value);
      counter++;
    };

  auto node = rclcpp::Node::make_shared("test_publisher_subscriber_serialized");
  auto subscriber = node->create_subscription<test_msgs::msg::BasicTypes>(
    "test_publisher_subscriber_serialized_topic", 10, serialized_callback);
  auto publisher = node->create_publisher<test_msgs::msg::BasicTypes>(
    "test_publisher_subscriber_serialized_topic", 10);

  test_msgs::msg::BasicTypes msg;

  rclcpp::Rate loop_rate(10);
  for (auto i = 0u; i < 10; ++i) {
    msg.uint8_value = i;
    publisher->publish(msg);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  EXPECT_GT(counter, 0u);
}
