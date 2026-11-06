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

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

using namespace std::chrono_literals;
using test_rosidl_buffer::msg::ByteArray;
using test_rosidl_buffer::TestBufferImpl;

class TestBufferPublisherGid : public ::testing::Test
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
};

// The backend must receive the endpoint info of the publisher, not of the
// DataWriter or key the RMW used to send the descriptor.
TEST_F(TestBufferPublisherGid, backend_receives_publisher_gid) {
  auto node = std::make_shared<rclcpp::Node>("test_buffer_publisher_gid");
  const auto qos = rclcpp::QoS(10).reliable();

  std::vector<ByteArray::SharedPtr> received;
  rclcpp::SubscriptionOptions options;
  options.acceptable_buffer_backends = "any";
  auto sub = node->create_subscription<ByteArray>(
    "test_buffer_publisher_gid", qos,
    [&received](ByteArray::SharedPtr msg) {received.push_back(msg);}, options);
  auto pub = node->create_publisher<ByteArray>("test_buffer_publisher_gid", qos);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spin_until = [&executor](auto condition, std::chrono::milliseconds timeout) {
      const auto end = std::chrono::steady_clock::now() + timeout;
      while (!condition() && std::chrono::steady_clock::now() < end) {
        executor.spin_some(10ms);
      }
    };
  spin_until([&pub]() {return pub->get_subscription_count() > 0;}, 10s);
  ASSERT_GT(pub->get_subscription_count(), 0u);
  // Give the buffer channels time to be discovered too.
  spin_until([]() {return false;}, 1s);

  ByteArray msg;
  msg.data = rosidl::Buffer<std::uint8_t>(
    std::make_unique<TestBufferImpl<std::uint8_t>>(std::vector<std::uint8_t>{1, 2, 3}));
  pub->publish(msg);
  spin_until([&received]() {return !received.empty();}, 10s);

  ASSERT_EQ(1u, received.size());
  ASSERT_EQ("test", received[0]->data.get_backend_type());
  const auto * impl =
    dynamic_cast<const TestBufferImpl<std::uint8_t> *>(received[0]->data.get_impl());
  ASSERT_NE(nullptr, impl);
  EXPECT_EQ(
    0, std::memcmp(impl->get_source_endpoint_gid().data(), pub->get_gid().data,
    RMW_GID_STORAGE_SIZE));
}
