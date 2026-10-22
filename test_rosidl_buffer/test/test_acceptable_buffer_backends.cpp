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

// rmw_subscription_options_t::acceptable_buffer_backends: the RMW must report
// an error when a requested backend is not installed.

#include <gtest/gtest.h>

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"

using test_rosidl_buffer::msg::ByteArray;

class TestAcceptableBufferBackends : public ::testing::Test
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
    node = std::make_shared<rclcpp::Node>("test_acceptable_buffer_backends");
  }

  rclcpp::Subscription<ByteArray>::SharedPtr create_subscription(const char * backends)
  {
    rclcpp::SubscriptionOptions options;
    options.acceptable_buffer_backends = backends;
    return node->create_subscription<ByteArray>(
      "test_acceptable_buffer_backends", 10, [](const ByteArray &) {}, options);
  }

  rclcpp::Node::SharedPtr node;
};

TEST_F(TestAcceptableBufferBackends, installed_backends) {
  for (const char * backends : {"", "cpu", "any", "test", "cpu,test", " test , cpu "}) {
    EXPECT_NO_THROW(create_subscription(backends)) << "acceptable_buffer_backends: " << backends;
  }
}

TEST_F(TestAcceptableBufferBackends, backend_not_installed) {
  for (const char * backends : {"nonexistent", "TEST", "test,nonexistent"}) {
    EXPECT_THROW(create_subscription(backends), rclcpp::exceptions::RCLError) <<
      "acceptable_buffer_backends: " << backends;
  }
}
