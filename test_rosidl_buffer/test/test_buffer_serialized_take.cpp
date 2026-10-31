// Copyright 2026 Open Source Robotics Foundation, Inc.
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

// Taking serialized messages from a subscription that accepts non-CPU buffer backends:
// messages received through a backend carry descriptors, so the take must fail, and the
// message must still be consumed.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

using namespace std::chrono_literals;
using test_rosidl_buffer::msg::ByteArray;
using test_rosidl_buffer::TestBufferImpl;

class TestBufferSerializedTake : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("test_buffer_serialized_take");
  }

  void create_endpoints(const char * topic, const char * backends)
  {
    rclcpp::SubscriptionOptions options;
    options.acceptable_buffer_backends = backends;
    sub = node->create_subscription<ByteArray>(
      topic, rclcpp::QoS(10).reliable(),
      [](std::shared_ptr<const rclcpp::SerializedMessage>) {}, options);
    pub = node->create_publisher<ByteArray>(topic, rclcpp::QoS(10).reliable());

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while ((sub->get_publisher_count() == 0 || pub->get_subscription_count() == 0) &&
      std::chrono::steady_clock::now() < deadline)
    {
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(pub->get_subscription_count(), 0u);
    // Let the buffer channels match too.
    std::this_thread::sleep_for(500ms);
  }

  void publish_test_buffer()
  {
    ByteArray msg;
    msg.seq = 1234;
    msg.data = rosidl::Buffer<std::uint8_t>(
      std::make_unique<TestBufferImpl<std::uint8_t>>(payload));
    pub->publish(msg);
  }

  bool wait_for_message(std::chrono::milliseconds timeout)
  {
    rclcpp::WaitSet wait_set({{sub}});
    return wait_set.wait(timeout).kind() == rclcpp::WaitResultKind::Ready;
  }

  const std::vector<std::uint8_t> payload = std::vector<std::uint8_t>(64, 7);
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<ByteArray>::SharedPtr sub;
  rclcpp::Publisher<ByteArray>::SharedPtr pub;
};

TEST_F(TestBufferSerializedTake, non_cpu_backend_take_fails_and_consumes_message) {
  create_endpoints("test_buffer_serialized_take_any", "any");
  publish_test_buffer();
  ASSERT_TRUE(wait_for_message(5s));

  rclcpp::SerializedMessage serialized;
  rclcpp::MessageInfo info;
  EXPECT_THROW(sub->take_serialized(serialized, info), rclcpp::exceptions::RCLError);
  rcutils_reset_error();

  // The message was consumed, so the subscription is not ready anymore.
  EXPECT_FALSE(wait_for_message(500ms));
}

TEST_F(TestBufferSerializedTake, cpu_backend_take_succeeds) {
  create_endpoints("test_buffer_serialized_take_cpu", "cpu");
  publish_test_buffer();
  ASSERT_TRUE(wait_for_message(5s));

  rclcpp::SerializedMessage serialized;
  rclcpp::MessageInfo info;
  ASSERT_TRUE(sub->take_serialized(serialized, info));

  ByteArray deserialized;
  rclcpp::Serialization<ByteArray>().deserialize_message(&serialized, &deserialized);
  EXPECT_EQ(1234u, deserialized.seq);
  const std::vector<std::uint8_t> & data = deserialized.data;
  EXPECT_EQ(payload, data);
}
