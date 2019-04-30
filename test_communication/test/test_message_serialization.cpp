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

#include "test_msgs/msg/bounded_sequences.h"
#include "test_msgs/msg/bounded_sequences.hpp"
#include "test_msgs/msg/basic_types.h"
#include "test_msgs/msg/basic_types.hpp"

#include "rcutils/allocator.h"

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

void fill_c_message(test_msgs__msg__BoundedSequences * bounded_sequences_msg_c)
{
  test_msgs__msg__BoundedSequences__init(bounded_sequences_msg_c);
  test_msgs__msg__BasicTypes__Sequence__init(&bounded_sequences_msg_c->basic_types_values, 1);
  bounded_sequences_msg_c->basic_types_values.data[0].bool_value = true;
  bounded_sequences_msg_c->basic_types_values.data[0].byte_value = 255;
  bounded_sequences_msg_c->basic_types_values.data[0].char_value = 'k';
  bounded_sequences_msg_c->basic_types_values.data[0].float32_value = 1;
  bounded_sequences_msg_c->basic_types_values.data[0].float64_value = 2;
  bounded_sequences_msg_c->basic_types_values.data[0].int8_value = 3;
  bounded_sequences_msg_c->basic_types_values.data[0].uint8_value = 4;
  bounded_sequences_msg_c->basic_types_values.data[0].int16_value = 5;
  bounded_sequences_msg_c->basic_types_values.data[0].uint16_value = 6;
  bounded_sequences_msg_c->basic_types_values.data[0].int32_value = 7;
  bounded_sequences_msg_c->basic_types_values.data[0].uint32_value = 8;
  bounded_sequences_msg_c->basic_types_values.data[0].int64_value = 9;
  bounded_sequences_msg_c->basic_types_values.data[0].uint64_value = 10;
}

void fill_cpp_message(test_msgs::msg::BoundedSequences * bounded_sequences_msg_cpp)
{
  test_msgs::msg::BasicTypes basic_types_msg_cpp;
  basic_types_msg_cpp.bool_value = true;
  basic_types_msg_cpp.byte_value = 255;
  basic_types_msg_cpp.char_value = 'k';
  basic_types_msg_cpp.float32_value = 1;
  basic_types_msg_cpp.float64_value = 2;
  basic_types_msg_cpp.int8_value = 3;
  basic_types_msg_cpp.uint8_value = 4;
  basic_types_msg_cpp.int16_value = 5;
  basic_types_msg_cpp.uint16_value = 6;
  basic_types_msg_cpp.int32_value = 7;
  basic_types_msg_cpp.uint32_value = 8;
  basic_types_msg_cpp.int64_value = 9;
  basic_types_msg_cpp.uint64_value = 10;
  bounded_sequences_msg_cpp->basic_basic_types_values.push_back(basic_types_msg_cpp);
}

TEST_F(CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION), de_serialize_c) {
  auto allocator = rcutils_get_default_allocator();

  auto serialized_message_c = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&serialized_message_c, 0, &allocator);
  ASSERT_EQ(RMW_RET_OK, ret);

  auto message_c_typesupport = ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BoundedSequences);
  test_msgs__msg__BoundedSequences bounded_sequences_msg_c;
  fill_c_message(&bounded_sequences_msg_c);

  ret = rmw_serialize(&bounded_sequences_msg_c, message_c_typesupport, &serialized_message_c);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(76u, serialized_message_c.buffer_length);  // measured from wireshark

  printf("serialized data length: %zu\n", serialized_message_c.buffer_length);
  print_serialized_buffer(serialized_message_c, "serialized message c");

  test_msgs__msg__BoundedSequences bounded_sequences_c_reverse;
  test_msgs__msg__BoundedSequences__init(&bounded_sequences_c_reverse);
  ret =
    rmw_deserialize(&serialized_message_c, message_c_typesupport, &bounded_sequences_c_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(true, bounded_sequences_c_reverse.basic_types_values.data[0].bool_value);
  EXPECT_EQ(255, bounded_sequences_c_reverse.basic_types_values.data[0].byte_value);
  EXPECT_EQ('k', bounded_sequences_c_reverse.basic_types_values.data[0].char_value);
  EXPECT_EQ(1, bounded_sequences_c_reverse.basic_types_values.data[0].float32_value);
  EXPECT_EQ(2, bounded_sequences_c_reverse.basic_types_values.data[0].float64_value);
  EXPECT_EQ(3, bounded_sequences_c_reverse.basic_types_values.data[0].int8_value);
  EXPECT_EQ(4u, bounded_sequences_c_reverse.basic_types_values.data[0].uint8_value);
  EXPECT_EQ(5, bounded_sequences_c_reverse.basic_types_values.data[0].int16_value);
  EXPECT_EQ(6u, bounded_sequences_c_reverse.basic_types_values.data[0].uint16_value);
  EXPECT_EQ(7, bounded_sequences_c_reverse.basic_types_values.data[0].int32_value);
  EXPECT_EQ(8u, bounded_sequences_c_reverse.basic_types_values.data[0].uint32_value);
  EXPECT_EQ(9, bounded_sequences_c_reverse.basic_types_values.data[0].int64_value);
  EXPECT_EQ(10u, bounded_sequences_c_reverse.basic_types_values.data[0].uint64_value);

  test_msgs__msg__BoundedSequences__fini(&bounded_sequences_c_reverse);
  test_msgs__msg__BoundedSequences__fini(&bounded_sequences_msg_c);
  ret = rmw_serialized_message_fini(&serialized_message_c);
  ASSERT_EQ(RMW_RET_OK, ret);
}

TEST_F(CLASSNAME(TestMessageSerialization, RMW_IMPLEMENTATION), de_serialize_cpp) {
  auto allocator = rcutils_get_default_allocator();

  auto serialized_message_cpp = rmw_get_zero_initialized_serialized_message();
  auto ret = rmw_serialized_message_init(&serialized_message_cpp, 0, &allocator);
  ASSERT_EQ(RMW_RET_OK, ret);

  auto message_cpp_typesupport =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::BoundedSequences>();

  test_msgs::msg::BoundedSequences bounded_sequences_msg_cpp;
  fill_cpp_message(&bounded_sequences_msg_cpp);

  ret =
    rmw_serialize(&bounded_sequences_msg_cpp, message_cpp_typesupport, &serialized_message_cpp);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(76u, serialized_message_cpp.buffer_length);
  print_serialized_buffer(serialized_message_cpp, "serialized message cpp");

  test_msgs::msg::BoundedSequences bounded_sequences_cpp_reverse;
  ret = rmw_deserialize(
    &serialized_message_cpp, message_cpp_typesupport, &bounded_sequences_cpp_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);
  EXPECT_EQ(true, bounded_sequences_cpp_reverse.basic_types_values[0].bool_value);
  EXPECT_EQ(255, bounded_sequences_cpp_reverse.basic_types_values[0].byte_value);
  EXPECT_EQ('k', bounded_sequences_cpp_reverse.basic_types_values[0].char_value);
  EXPECT_EQ(1, bounded_sequences_cpp_reverse.basic_types_values[0].float32_value);
  EXPECT_EQ(2, bounded_sequences_cpp_reverse.basic_types_values[0].float64_value);
  EXPECT_EQ(3, bounded_sequences_cpp_reverse.basic_types_values[0].int8_value);
  EXPECT_EQ(4u, bounded_sequences_cpp_reverse.basic_types_values[0].uint8_value);
  EXPECT_EQ(5, bounded_sequences_cpp_reverse.basic_types_values[0].int16_value);
  EXPECT_EQ(6u, bounded_sequences_cpp_reverse.basic_types_values[0].uint16_value);
  EXPECT_EQ(7, bounded_sequences_cpp_reverse.basic_types_values[0].int32_value);
  EXPECT_EQ(8u, bounded_sequences_cpp_reverse.basic_types_values[0].uint32_value);
  EXPECT_EQ(9, bounded_sequences_cpp_reverse.basic_types_values[0].int64_value);
  EXPECT_EQ(10u, bounded_sequences_cpp_reverse.basic_types_values[0].uint64_value);

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

  auto message_c_typesupport = ROSIDL_GET_MSG_TYPE_SUPPORT(test_msgs, msg, BoundedSequences);
  auto message_cpp_typesupport =
    rosidl_typesupport_cpp::get_message_type_support_handle<test_msgs::msg::BoundedSequences>();

  test_msgs__msg__BoundedSequences msg_c;
  test_msgs__msg__BoundedSequences__init(&msg_c);
  fill_c_message(&msg_c);

  test_msgs::msg::BoundedSequences msg_cpp;
  fill_cpp_message(&msg_cpp);

  ret = rmw_serialize(&msg_c, message_c_typesupport, &serialized_message_c);
  ASSERT_EQ(RMW_RET_OK, ret);
  ret = rmw_serialize(&msg_cpp, message_cpp_typesupport, &serialized_message_cpp);
  EXPECT_EQ(RMW_RET_OK, ret);

  print_serialized_buffer(serialized_message_c, "serialized message c");
  print_serialized_buffer(serialized_message_cpp, "serialized message cpp");

  test_msgs__msg__BoundedSequences msg_c_reverse;
  test_msgs__msg__BoundedSequences__init(&msg_c_reverse);
  test_msgs::msg::BoundedSequences msg_cpp_reverse;
  ret = rmw_deserialize(&serialized_message_cpp, message_c_typesupport, &msg_c_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);
  ret = rmw_deserialize(&serialized_message_c, message_cpp_typesupport, &msg_cpp_reverse);
  EXPECT_EQ(RMW_RET_OK, ret);

  test_msgs__msg__BoundedSequences__fini(&msg_c);
  test_msgs__msg__BoundedSequences__fini(&msg_c_reverse);
  ret = rmw_serialized_message_fini(&serialized_message_c);
  ASSERT_EQ(RMW_RET_OK, ret);
  ret = rmw_serialized_message_fini(&serialized_message_cpp);
  ASSERT_EQ(RMW_RET_OK, ret);
}
