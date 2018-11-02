// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <string>

#include "rmw/serialized_message.h"
#include "rmw/rmw.h"

#include "test_msgs/msg/bounded_array_nested.h"
#include "test_msgs/msg/bounded_array_nested.hpp"
#include "test_msgs/msg/primitives.h"
#include "test_msgs/msg/primitives.hpp"

#include "rcutils/allocator.h"

#include "rosidl_generator_c/string_functions.h"

#include "rosidl_typesupport_cpp/message_type_support.hpp"

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
  }
};

void print_serialized_buffer(
  const rmw_serialized_message_t & serialized_msg, std::string prefix = "")
{
  printf("%s\n", prefix.c_str());
  for (unsigned int i = 0; i < serialized_msg.buffer_length; ++i) {
    printf("%02x ", serialized_msg.buffer[i]);
  }
  printf("\n");
}

void fill_c_message(test_msgs__msg__BoundedArrayNested * bounded_array_nested_msg_c)
{
  test_msgs__msg__BoundedArrayNested__init(bounded_array_nested_msg_c);
  test_msgs__msg__Primitives__Sequence__init(&bounded_array_nested_msg_c->primitive_values, 1);
  bounded_array_nested_msg_c->primitive_values.data[0].bool_value = true;
  bounded_array_nested_msg_c->primitive_values.data[0].byte_value = 255;
  bounded_array_nested_msg_c->primitive_values.data[0].char_value = 'k';
  bounded_array_nested_msg_c->primitive_values.data[0].float32_value = 1;
  bounded_array_nested_msg_c->primitive_values.data[0].float64_value = 2;
  bounded_array_nested_msg_c->primitive_values.data[0].int8_value = 3;
  bounded_array_nested_msg_c->primitive_values.data[0].uint8_value = 4;
  bounded_array_nested_msg_c->primitive_values.data[0].int16_value = 5;
  bounded_array_nested_msg_c->primitive_values.data[0].uint16_value = 6;
  bounded_array_nested_msg_c->primitive_values.data[0].int32_value = 7;
  bounded_array_nested_msg_c->primitive_values.data[0].uint32_value = 8;
  bounded_array_nested_msg_c->primitive_values.data[0].int64_value = 9;
  bounded_array_nested_msg_c->primitive_values.data[0].uint64_value = 10;
  rosidl_generator_c__String__assign(
    &bounded_array_nested_msg_c->primitive_values.data[0].string_value, "hello world");
}

void fill_cpp_message(test_msgs::msg::BoundedArrayNested * bounded_array_nested_msg_cpp)
{
  test_msgs::msg::Primitives primitive_msg_cpp;
  primitive_msg_cpp.bool_value = true;
  primitive_msg_cpp.byte_value = 255;
  primitive_msg_cpp.char_value = 'k';
  primitive_msg_cpp.float32_value = 1;
  primitive_msg_cpp.float64_value = 2;
  primitive_msg_cpp.int8_value = 3;
  primitive_msg_cpp.uint8_value = 4;
  primitive_msg_cpp.int16_value = 5;
  primitive_msg_cpp.uint16_value = 6;
  primitive_msg_cpp.int32_value = 7;
  primitive_msg_cpp.uint32_value = 8;
  primitive_msg_cpp.int64_value = 9;
  primitive_msg_cpp.uint64_value = 10;
  primitive_msg_cpp.string_value = "hello world";
  bounded_array_nested_msg_cpp->primitive_values.push_back(primitive_msg_cpp);
}

TEST_F(CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION), de_serialize_c) {
  auto allocator = rcutils_get_default_allocator();

  auto serialized_message_c = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&serialized_message_c, 0, &allocator);
  ASSERT_EQ(RMW_RET_OK, ret);

  auto message_c_typesupport = ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BoundedArrayNested);
  test_msgs__msg__BoundedArrayNested bounded_array_nested_msg_c;
  fill_c_message(&bounded_array_nested_msg_c);

  ret = rmw_serialize(&bounded_array_nested_msg_c, message_c_typesupport, &serialized_message_c);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(76u, serialized_message_c.buffer_length);  // measured from wireshark

  printf("serialized data length: %zu\n", serialized_message_c.buffer_length);
  print_serialized_buffer(serialized_message_c, "serialized message c");

  test_msgs__msg__BoundedArrayNested bounded_array_nested_c_reverse;
  test_msgs__msg__BoundedArrayNested__init(&bounded_array_nested_c_reverse);
  ret =
    rmw_deserialize(&serialized_message_c, message_c_typesupport, &bounded_array_nested_c_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(true, bounded_array_nested_c_reverse.primitive_values.data[0].bool_value);
  EXPECT_EQ(255, bounded_array_nested_c_reverse.primitive_values.data[0].byte_value);
  EXPECT_EQ('k', bounded_array_nested_c_reverse.primitive_values.data[0].char_value);
  EXPECT_EQ(1, bounded_array_nested_c_reverse.primitive_values.data[0].float32_value);
  EXPECT_EQ(2, bounded_array_nested_c_reverse.primitive_values.data[0].float64_value);
  EXPECT_EQ(3, bounded_array_nested_c_reverse.primitive_values.data[0].int8_value);
  EXPECT_EQ(4u, bounded_array_nested_c_reverse.primitive_values.data[0].uint8_value);
  EXPECT_EQ(5, bounded_array_nested_c_reverse.primitive_values.data[0].int16_value);
  EXPECT_EQ(6u, bounded_array_nested_c_reverse.primitive_values.data[0].uint16_value);
  EXPECT_EQ(7, bounded_array_nested_c_reverse.primitive_values.data[0].int32_value);
  EXPECT_EQ(8u, bounded_array_nested_c_reverse.primitive_values.data[0].uint32_value);
  EXPECT_EQ(9, bounded_array_nested_c_reverse.primitive_values.data[0].int64_value);
  EXPECT_EQ(10u, bounded_array_nested_c_reverse.primitive_values.data[0].uint64_value);
  EXPECT_STREQ(
    "hello world", bounded_array_nested_c_reverse.primitive_values.data[0].string_value.data);

  test_msgs__msg__BoundedArrayNested__fini(&bounded_array_nested_c_reverse);
  test_msgs__msg__BoundedArrayNested__fini(&bounded_array_nested_msg_c);
  ret = rmw_serialized_message_fini(&serialized_message_c);
  ASSERT_EQ(RMW_RET_OK, ret);
}

TEST_F(CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION), de_serialize_cpp) {
  auto allocator = rcutils_get_default_allocator();

  auto serialized_message_cpp = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&serialized_message_cpp, 0, &allocator);
  ASSERT_EQ(RMW_RET_OK, ret);

  auto message_cpp_typesupport =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::BoundedArrayNested>();

  test_msgs::msg::BoundedArrayNested bounded_array_nested_msg_cpp;
  fill_cpp_message(&bounded_array_nested_msg_cpp);

  ret =
    rmw_serialize(&bounded_array_nested_msg_cpp, message_cpp_typesupport, &serialized_message_cpp);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(76u, serialized_message_cpp.buffer_length);
  print_serialized_buffer(serialized_message_cpp, "serialized message cpp");

  test_msgs::msg::BoundedArrayNested bounded_array_nested_cpp_reverse;
  ret = rmw_deserialize(
    &serialized_message_cpp, message_cpp_typesupport, &bounded_array_nested_cpp_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(true, bounded_array_nested_cpp_reverse.primitive_values[0].bool_value);
  EXPECT_EQ(255, bounded_array_nested_cpp_reverse.primitive_values[0].byte_value);
  EXPECT_EQ('k', bounded_array_nested_cpp_reverse.primitive_values[0].char_value);
  EXPECT_EQ(1, bounded_array_nested_cpp_reverse.primitive_values[0].float32_value);
  EXPECT_EQ(2, bounded_array_nested_cpp_reverse.primitive_values[0].float64_value);
  EXPECT_EQ(3, bounded_array_nested_cpp_reverse.primitive_values[0].int8_value);
  EXPECT_EQ(4u, bounded_array_nested_cpp_reverse.primitive_values[0].uint8_value);
  EXPECT_EQ(5, bounded_array_nested_cpp_reverse.primitive_values[0].int16_value);
  EXPECT_EQ(6u, bounded_array_nested_cpp_reverse.primitive_values[0].uint16_value);
  EXPECT_EQ(7, bounded_array_nested_cpp_reverse.primitive_values[0].int32_value);
  EXPECT_EQ(8u, bounded_array_nested_cpp_reverse.primitive_values[0].uint32_value);
  EXPECT_EQ(9, bounded_array_nested_cpp_reverse.primitive_values[0].int64_value);
  EXPECT_EQ(10u, bounded_array_nested_cpp_reverse.primitive_values[0].uint64_value);
  EXPECT_STREQ(
    "hello world", bounded_array_nested_cpp_reverse.primitive_values[0].string_value.c_str());

  ret = rmw_serialized_message_fini(&serialized_message_cpp);
  ASSERT_EQ(RMW_RET_OK, ret);
}

TEST_F(CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION), cdr_integrity) {
  auto allocator = rcutils_get_default_allocator();

  auto serialized_message_c = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&serialized_message_c, 0, &allocator);
  ASSERT_EQ(RMW_RET_OK, ret);

  auto serialized_message_cpp = rmw_get_zero_initialized_serialized_message();
  ret = rmw_serialized_message_init(&serialized_message_cpp, 0, &allocator);
  ASSERT_EQ(RMW_RET_OK, ret);

  auto message_c_typesupport = ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BoundedArrayNested);
  auto message_cpp_typesupport =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::BoundedArrayNested>();

  test_msgs__msg__BoundedArrayNested msg_c;
  test_msgs__msg__BoundedArrayNested__init(&msg_c);
  fill_c_message(&msg_c);

  test_msgs::msg::BoundedArrayNested msg_cpp;
  fill_cpp_message(&msg_cpp);

  ret = rmw_serialize(&msg_c, message_c_typesupport, &serialized_message_c);
  ASSERT_EQ(RMW_RET_OK, ret);
  ret = rmw_serialize(&msg_cpp, message_cpp_typesupport, &serialized_message_cpp);
  EXPECT_EQ(RMW_RET_OK, ret);

  print_serialized_buffer(serialized_message_c, "serialized message c");
  print_serialized_buffer(serialized_message_cpp, "serialized message cpp");

  test_msgs__msg__BoundedArrayNested msg_c_reverse;
  test_msgs__msg__BoundedArrayNested__init(&msg_c_reverse);
  test_msgs::msg::BoundedArrayNested msg_cpp_reverse;
  ret = rmw_deserialize(&serialized_message_cpp, message_c_typesupport, &msg_c_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);
  ret = rmw_deserialize(&serialized_message_c, message_cpp_typesupport, &msg_cpp_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);

  test_msgs__msg__BoundedArrayNested__fini(&msg_c);
  test_msgs__msg__BoundedArrayNested__fini(&msg_c_reverse);
  ret = rmw_serialized_message_fini(&serialized_message_c);
  ASSERT_EQ(RMW_RET_OK, ret);
  ret = rmw_serialized_message_fini(&serialized_message_cpp);
  ASSERT_EQ(RMW_RET_OK, ret);
}
