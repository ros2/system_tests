// Copyright 2016 Open Source Robotics Foundation, Inc.
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
#include <string>
#include <thread>

#include "rcl/subscription.h"
#include "rcl/publisher.h"

#include "rcl/rcl.h"

#include "test_communication/msg/dynamic_array_nested.h"
#include "test_communication/msg/dynamic_array_primitives.h"
#include "test_communication/msg/empty.h"
#include "test_communication/msg/nested.h"
#include "test_communication/msg/primitives.h"
#include "test_communication/msg/static_array_nested.h"
#include "test_communication/msg/static_array_primitives.h"

#include "rosidl_generator_c/string_functions.h"
#include "rosidl_generator_c/primitives_array_functions.h"
#include "rosidl_generator_c/message_type_support.h"
#include "rcl/error_handling.h"

#ifndef SCOPE_EXIT_HPP_
#define SCOPE_EXIT_HPP_

#include <functional>

template<typename Callable>
struct ScopeExit
{
  explicit ScopeExit(Callable callable)
  : callable_(callable) {}
  ~ScopeExit() {callable_(); }

private:
  Callable callable_;
};

template<typename Callable>
ScopeExit<Callable>
make_scope_exit(Callable callable)
{
  return ScopeExit<Callable>(callable);
}

#define SCOPE_EXIT(code) make_scope_exit([&]() {code; })

#endif  // SCOPE_EXIT_HPP_


#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestMessagesFixture, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  rcl_node_t * node_ptr;
  void SetUp()
  {
    rcl_ret_t ret;
    ret = rcl_init(0, nullptr, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();
    const char * name = "node_name";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node_ptr, name, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  }

  void TearDown()
  {
    rcl_ret_t ret = rcl_node_fini(this->node_ptr);
    delete this->node_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_shutdown();
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  }

  template<typename MessageT>
  void test_message_type(const char * topic_name, const rosidl_message_type_support_t * ts)
  {
    rcl_ret_t ret;
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();

    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    ret = rcl_publisher_init(&publisher, this->node_ptr, ts, topic_name, &publisher_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    auto publisher_exit = make_scope_exit([&publisher, this]() {
      rcl_ret_t ret = rcl_publisher_fini(&publisher, this->node_ptr);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    });
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    ret = rcl_subscription_init(
      &subscription, this->node_ptr, ts, topic_name, &subscription_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    auto subscription_exit = make_scope_exit([&subscription, this]() {
      rcl_ret_t ret = rcl_subscription_fini(&subscription, this->node_ptr);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    });
    EXPECT_EQ(strcmp(rcl_subscription_get_topic_name(&subscription), topic_name), 0);
    // TODO(wjwwood): add logic to wait for the connection to be established
    //                probably using the count_subscriptions busy wait mechanism
    //                until then we will sleep for a short period of time
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      MessageT message;
      init_message(&message);
      auto msg_exit = make_scope_exit([&message]() {
        fini_message(&message);
      });
      ret = rcl_publish(&publisher, &message);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    {
      MessageT message;
      init_message(&message);
      auto msg_exit = make_scope_exit([&message]() {
        fini_message(&message);
      });
      ret = rcl_take(&subscription, &message, nullptr);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      verify_message(message);
    }
  }
};


// Utilities for test fixtures

template<typename MessageT>
void init_message(MessageT * msg);

template<typename MessageT>
void verify_message(MessageT & msg);

template<typename MessageT>
void fini_message(MessageT * msg);

#define DEFINE_FINI_MESSAGE(TYPE) \
template<> \
void fini_message(TYPE * msg) { \
  TYPE ## __fini(msg); \
}

// Test publish/subscribe with a variety of messages


// Define functions and test cases for each message type

// test_communication__msg__Primitives
template<>
void init_message(test_communication__msg__Primitives * message)
{
  test_communication__msg__Primitives__init(message);
  message->bool_value = true;
  message->byte_value = '1';
  message->char_value = '2';
  message->float32_value = 1.5717;
  message->float64_value = 3.14159;
  message->int8_value = -7;
  message->uint8_value = 7;
  message->int16_value = -4;
  message->uint16_value = 5;
  message->int32_value = -6;
  message->uint32_value = 6;
  message->int64_value = -8;
  message->uint64_value = 8;
  // rosidl_generator_c__String__init(&message->string_value);
  rosidl_generator_c__String__assign(&message->string_value, "hello world");
}

template<>
void verify_message(test_communication__msg__Primitives & message)
{
  EXPECT_EQ(message.bool_value, true);
  EXPECT_EQ(message.byte_value, '1');
  EXPECT_EQ(message.char_value, '2');
  EXPECT_FLOAT_EQ(message.float32_value, 1.5717);
  EXPECT_DOUBLE_EQ(message.float64_value, 3.14159);
  EXPECT_EQ(message.int8_value, -7);
  EXPECT_EQ(message.uint8_value, 7);
  EXPECT_EQ(message.int16_value, -4);
  EXPECT_EQ(message.uint16_value, 5);
  EXPECT_EQ(message.int32_value, -6);
  EXPECT_EQ(message.uint32_value, 6);
  EXPECT_EQ(message.int64_value, -8);
  EXPECT_EQ(message.uint64_value, 8);
  EXPECT_EQ(strcmp(message.string_value.data, "hello world"), 0);
}

DEFINE_FINI_MESSAGE(test_communication__msg__Primitives);
TEST_F(CLASSNAME (TestMessagesFixture, RMW_IMPLEMENTATION), test_primitives)
{
  const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_message_type<test_communication__msg__Primitives>("test_primitives", ts);
}

// test_communication__msg__DynamicArrayNested
template<>
void init_message(test_communication__msg__DynamicArrayNested * message)
{
  test_communication__msg__DynamicArrayNested__init(message);
  test_communication__msg__Primitives__Array__init(
    &message->primitive_values, 3);
  for (size_t i = 0; i < 3; ++i) {
    init_message(&message->primitive_values.data[i]);
  }
}

template<>
void verify_message(test_communication__msg__DynamicArrayNested & message)
{
  EXPECT_EQ(message.primitive_values.size, 3);
  for (size_t i = 0; i < 3; ++i) {
    verify_message(message.primitive_values.data[i]);
  }
}

DEFINE_FINI_MESSAGE(test_communication__msg__DynamicArrayNested);

TEST_F(CLASSNAME (TestMessagesFixture, RMW_IMPLEMENTATION), test_dynamic_array_nested)
{
  const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
    test_communication, msg, DynamicArrayNested);
  test_message_type<test_communication__msg__DynamicArrayNested>("test_dynamic_array_nested", ts);
}

// test_communication__msg__DynamicArrayPrimitives
template<>
void init_message(test_communication__msg__DynamicArrayPrimitives * message)
{
  test_communication__msg__DynamicArrayPrimitives__init(message);
  rosidl_generator_c__bool__Array__init(&message->bool_values, 3);
  rosidl_generator_c__byte__Array__init(&message->byte_values, 3);
  rosidl_generator_c__char__Array__init(&message->char_values, 3);
  rosidl_generator_c__float32__Array__init(&message->float32_values, 3);
  rosidl_generator_c__float64__Array__init(&message->float64_values, 3);
  rosidl_generator_c__int8__Array__init(&message->int8_values, 3);
  rosidl_generator_c__uint8__Array__init(&message->uint8_values, 3);
  rosidl_generator_c__int16__Array__init(&message->int16_values, 3);
  rosidl_generator_c__uint16__Array__init(&message->uint16_values, 3);
  rosidl_generator_c__int32__Array__init(&message->int32_values, 3);
  rosidl_generator_c__uint32__Array__init(&message->uint32_values, 3);
  rosidl_generator_c__int64__Array__init(&message->int64_values, 3);
  rosidl_generator_c__uint64__Array__init(&message->uint64_values, 3);
  rosidl_generator_c__String__Array__init(&message->string_values, 3);
  for (size_t i = 0; i < 3; ++i) {
    message->bool_values.data[i] = true;
    message->byte_values.data[i] = '1';
    message->char_values.data[i] = '2';
    message->float32_values.data[i] = 1.5717;
    message->float64_values.data[i] = 3.14159;
    message->int8_values.data[i] = -7;
    message->uint8_values.data[i] = 7;
    message->int16_values.data[i] = -4;
    message->uint16_values.data[i] = 5;
    message->int32_values.data[i] = -6;
    message->uint32_values.data[i] = 6;
    message->int64_values.data[i] = -8;
    message->uint64_values.data[i] = 8;
    // rosidl_generator_c__String__init(&message->string_values.data[i]);
    rosidl_generator_c__String__assign(&message->string_values.data[i], "hello world");
  }
}

template<>
void verify_message(test_communication__msg__DynamicArrayPrimitives & message)
{
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(message.bool_values.data[i], true);
    EXPECT_EQ(message.byte_values.data[i], '1');
    EXPECT_EQ(message.char_values.data[i], '2');
    EXPECT_FLOAT_EQ(message.float32_values.data[i], 1.5717);
    EXPECT_DOUBLE_EQ(message.float64_values.data[i], 3.14159);
    EXPECT_EQ(message.int8_values.data[i], -7);
    EXPECT_EQ(message.uint8_values.data[i], 7);
    EXPECT_EQ(message.int16_values.data[i], -4);
    EXPECT_EQ(message.uint16_values.data[i], 5);
    EXPECT_EQ(message.int32_values.data[i], -6);
    EXPECT_EQ(message.uint32_values.data[i], 6);
    EXPECT_EQ(message.int64_values.data[i], -8);
    EXPECT_EQ(message.uint64_values.data[i], 8);
    EXPECT_EQ(strcmp(message.string_values.data[i].data, "hello world"), 0);
  }
}

DEFINE_FINI_MESSAGE(test_communication__msg__DynamicArrayPrimitives);

TEST_F(CLASSNAME (TestMessagesFixture, RMW_IMPLEMENTATION), test_dynamic_array_primitives)
{
  const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
    test_communication, msg, DynamicArrayPrimitives);
  test_message_type<test_communication__msg__DynamicArrayPrimitives>(
    "test_dynamic_array_primitives", ts);
}

// test_communication__msg__StaticArrayPrimitives
template<>
void init_message(test_communication__msg__StaticArrayPrimitives * message)
{
  test_communication__msg__StaticArrayPrimitives__init(message);
  for (size_t i = 0; i < 3; ++i) {
    message->bool_values[i] = true;
    message->byte_values[i] = '1';
    message->char_values[i] = '2';
    message->float32_values[i] = 1.5717;
    message->float64_values[i] = 3.14159;
    message->int8_values[i] = -7;
    message->uint8_values[i] = 7;
    message->int16_values[i] = -4;
    message->uint16_values[i] = 5;
    message->int32_values[i] = -6;
    message->uint32_values[i] = 6;
    message->int64_values[i] = -8;
    message->uint64_values[i] = 8;
    // rosidl_generator_c__String__init(&message->string_values[i]);
    rosidl_generator_c__String__assign(&message->string_values[i], "hello world");
  }
}

template<>
void verify_message(test_communication__msg__StaticArrayPrimitives & message)
{
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(message.bool_values[i], true);
    EXPECT_EQ(message.byte_values[i], '1');
    EXPECT_EQ(message.char_values[i], '2');
    EXPECT_FLOAT_EQ(message.float32_values[i], 1.5717);
    EXPECT_DOUBLE_EQ(message.float64_values[i], 3.14159);
    EXPECT_EQ(message.int8_values[i], -7);
    EXPECT_EQ(message.uint8_values[i], 7);
    EXPECT_EQ(message.int16_values[i], -4);
    EXPECT_EQ(message.uint16_values[i], 5);
    EXPECT_EQ(message.int32_values[i], -6);
    EXPECT_EQ(message.uint32_values[i], 6);
    EXPECT_EQ(message.int64_values[i], -8);
    EXPECT_EQ(message.uint64_values[i], 8);
    EXPECT_EQ(strcmp(message.string_values[i].data, "hello world"), 0);
  }
}

DEFINE_FINI_MESSAGE(test_communication__msg__StaticArrayPrimitives);

TEST_F(CLASSNAME (TestMessagesFixture, RMW_IMPLEMENTATION), test_static_array_primitives)
{
  const rosidl_message_type_support_t * ts = ROSIDL_GET_TYPE_SUPPORT(
    test_communication, msg, StaticArrayPrimitives);
  test_message_type<test_communication__msg__StaticArrayPrimitives>(
    "test_static_array_primitives", ts);
}


// test_communication__msg__StaticArrayNested

