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

#include <cstdint>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

using test_rosidl_buffer::msg::ByteArray;
using test_rosidl_buffer::TestBufferImpl;

class TestBufferSerialize : public ::testing::Test
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

// Serializing a non-CPU Buffer writes CPU bytes, so it must give the same
// length as the same data in a CPU Buffer.
TEST_F(TestBufferSerialize, non_cpu_buffer_has_cpu_serialized_length) {
  const std::vector<std::uint8_t> payload(64, 7);
  rclcpp::Serialization<ByteArray> serialization;

  ByteArray cpu_msg;
  cpu_msg.seq = 1234;
  cpu_msg.data = payload;
  rclcpp::SerializedMessage cpu_serialized;
  serialization.serialize_message(&cpu_msg, &cpu_serialized);

  ByteArray test_msg;
  test_msg.seq = 1234;
  test_msg.data = rosidl::Buffer<std::uint8_t>(
    std::make_unique<TestBufferImpl<std::uint8_t>>(payload));
  ASSERT_EQ("test", test_msg.data.get_backend_type());
  rclcpp::SerializedMessage test_serialized;
  serialization.serialize_message(&test_msg, &test_serialized);

  EXPECT_EQ(cpu_serialized.size(), test_serialized.size());

  ByteArray deserialized;
  serialization.deserialize_message(&test_serialized, &deserialized);
  EXPECT_EQ(1234u, deserialized.seq);
  const std::vector<std::uint8_t> & data = deserialized.data;
  EXPECT_EQ(payload, data);
}
