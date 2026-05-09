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
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"

#include "rmw/types.h"

#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/unbounded_sequences.hpp"

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
  bool got_first_msg = false;
  size_t counter = 0;

  auto serialized_callback =
    [&got_first_msg,
      &counter](const std::shared_ptr<const rclcpp::SerializedMessage> serialized_msg) {
      using MessageT = test_msgs::msg::BasicTypes;
      rclcpp::Serialization<MessageT> serializer;
      MessageT basic_types_msg;
      EXPECT_NO_THROW(serializer.deserialize_message(serialized_msg.get(), &basic_types_msg));

      if (!got_first_msg) {
        // start counting from first message received, in case we miss the first couple messages
        counter = basic_types_msg.uint8_value;
        got_first_msg = true;
      }

      printf("received message %zu\n", counter);
      for (auto i = 0u; i < serialized_msg->size(); ++i) {
        printf("%02x ", serialized_msg->get_rcl_serialized_message().buffer[i]);
      }
      printf("\n");

      EXPECT_EQ(counter, basic_types_msg.uint8_value);
      counter++;
    };

  auto node = rclcpp::Node::make_shared("test_publisher_subscriber_serialized");
  auto subscriber = node->create_subscription<test_msgs::msg::BasicTypes>(
    "test_publisher_subscriber_serialized_topic", 10, serialized_callback);
  auto publisher = node->create_publisher<test_msgs::msg::BasicTypes>(
    "test_publisher_subscriber_serialized_topic", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  test_msgs::msg::BasicTypes msg;

  rclcpp::Rate loop_rate(10);
  for (auto i = 0u; i < 10; ++i) {
    msg.uint8_value = i;
    publisher->publish(msg);
    executor.spin_some();
    loop_rate.sleep();
  }
  EXPECT_GT(counter, 0u);
}

// Regression test for buffer-aware serialized take. Messages with `uint8[]`
// fields put rmw_fastrtps publishers on a separate "shared CPU channel"
// DataWriter for CPU-only subscribers; rmw_take_serialized_message must
// dispatch onto that path or generic/serialized subscribers (foxglove_bridge,
// ros2 bag record, rclcpp::GenericSubscription) silently get no data.
//
// Subscription options are intentionally left at defaults: the broken path
// is the one taken by every generic/serialized consumer today, which is
// `acceptable_buffer_backends` unset (=> CPU-only buffer-aware subscriber).
TEST_F(
  CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION),
  serialized_callback_buffer_aware_message)
{
  using MessageT = test_msgs::msg::UnboundedSequences;
  const std::vector<uint8_t> expected_bytes{1, 2, 3, 4};

  size_t received = 0;
  auto serialized_callback =
    [&received, &expected_bytes](
    const std::shared_ptr<const rclcpp::SerializedMessage> serialized_msg) {
      rclcpp::Serialization<MessageT> serializer;
      MessageT decoded;
      EXPECT_NO_THROW(serializer.deserialize_message(serialized_msg.get(), &decoded));
      EXPECT_EQ(decoded.uint8_values, expected_bytes);
      ++received;
    };

  auto node = rclcpp::Node::make_shared("test_publisher_subscriber_serialized_buffer");
  auto subscriber = node->create_subscription<MessageT>(
    "test_publisher_subscriber_serialized_buffer_topic", 10, serialized_callback);
  auto publisher = node->create_publisher<MessageT>(
    "test_publisher_subscriber_serialized_buffer_topic", 10);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  MessageT msg;
  msg.uint8_values = expected_bytes;

  rclcpp::Rate loop_rate(10);
  for (auto i = 0u; i < 10; ++i) {
    publisher->publish(msg);
    executor.spin_some();
    loop_rate.sleep();
  }
  EXPECT_GT(received, 0u);
}
