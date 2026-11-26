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

// Endpoint-aware deserialization of a message whose nested message holds a
// Buffer descriptor that cannot be resolved must fail, not continue reading
// the following fields from the wrong bytes.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <string>

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"
#include "rmw/topic_endpoint_info.h"
#include "rosidl_typesupport_fastrtps_cpp/buffer_serialization.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_interface/macros.h"
#include "test_rosidl_buffer/msg/byte_array_list.h"
#include "test_rosidl_buffer/msg/byte_array_list.hpp"

extern "C"
{
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
  rosidl_typesupport_fastrtps_c, test_rosidl_buffer, msg, ByteArrayList)();
}

namespace
{

// Deserializes a ByteArrayList with one item whose data is a descriptor for a
// backend that is not loaded.
bool deserialize_unresolvable_item(
  const rosidl_message_type_support_t * type_support, void * ros_message)
{
  std::array<char, 256> raw{};
  eprosima::fastcdr::FastBuffer fast_buffer(raw.data(), raw.size());
  {
    eprosima::fastcdr::Cdr cdr(fast_buffer);
    cdr << static_cast<uint32_t>(1);  // items
    cdr << static_cast<uint32_t>(7);  // items[0].seq
    cdr << rosidl_typesupport_fastrtps_cpp::kBufferDescriptorMarker1;
    cdr << rosidl_typesupport_fastrtps_cpp::kBufferDescriptorMarker2;
    cdr << std::string("nonexistent");
    cdr << static_cast<uint64_t>(42);
  }

  eprosima::fastcdr::Cdr cdr(fast_buffer);
  auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
  const auto endpoint_info = rmw_get_zero_initialized_topic_endpoint_info();
  rosidl_typesupport_fastrtps_cpp::BufferSerializationContext serialization_context;
  return callbacks->cdr_deserialize_with_endpoint(
    cdr, ros_message, endpoint_info, serialization_context);
}

}  // namespace

TEST(TestNestedBufferDeserialize, cpp_nested_failure_fails_message) {
  test_rosidl_buffer::msg::ByteArrayList msg;
  EXPECT_FALSE(
    deserialize_unresolvable_item(
      rosidl_typesupport_fastrtps_cpp::get_message_type_support_handle<
        test_rosidl_buffer::msg::ByteArrayList>(),
      &msg));
}

TEST(TestNestedBufferDeserialize, c_nested_failure_fails_message) {
  test_rosidl_buffer__msg__ByteArrayList msg;
  ASSERT_TRUE(test_rosidl_buffer__msg__ByteArrayList__init(&msg));
  EXPECT_FALSE(
    deserialize_unresolvable_item(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, test_rosidl_buffer, msg, ByteArrayList)(),
      &msg));
  test_rosidl_buffer__msg__ByteArrayList__fini(&msg);
}
