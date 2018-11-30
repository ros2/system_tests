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

#include <stdio.h>
#include <stdlib.h>

#include <gtest/gtest.h>

#include <chrono>
#include <string>
#include <thread>

#include "osrf_testing_tools_cpp/scope_exit.hpp"

#include "rcl/subscription.h"
#include "rcl/publisher.h"

#include "rcl/rcl.h"

#include "test_msgs/msg/bounded_array_nested.h"
#include "test_msgs/msg/bounded_array_primitives.h"
#include "test_msgs/msg/dynamic_array_nested.h"
#include "test_msgs/msg/dynamic_array_primitives.h"
#include "test_msgs/msg/dynamic_array_primitives_nested.h"
#include "test_msgs/msg/empty.h"
#include "test_msgs/msg/nested.h"
#include "test_msgs/msg/primitives.h"
#include "test_msgs/msg/static_array_nested.h"
#include "test_msgs/msg/static_array_primitives.h"
#include "test_msgs/msg/builtins.h"

#include "rosidl_generator_c/string_functions.h"
#include "rosidl_generator_c/primitives_sequence_functions.h"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rcl/error_handling.h"

#ifndef SCOPE_EXIT_HPP_
#define SCOPE_EXIT_HPP_

#include <functional>
#include <limits>
#include <memory>

template<typename Callable>
struct ScopeExit
{
  explicit ScopeExit(Callable callable)
  : callable_(callable) {}
  ~ScopeExit() {callable_();}

private:
  Callable callable_;
};

template<typename Callable>
ScopeExit<Callable>
make_scope_exit(Callable callable)
{
  return ScopeExit<Callable>(callable);
}

#define SCOPE_EXIT(code) make_scope_exit([&]() {code;})

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
  rcl_context_t * context_ptr;
  rcl_node_t * node_ptr;
  void SetUp()
  {
    rcl_ret_t ret;
    {
      rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
      ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      OSRF_TESTING_TOOLS_CPP_SCOPE_EXIT({
        EXPECT_EQ(RCL_RET_OK, rcl_init_options_fini(&init_options)) << rcl_get_error_string().str;
      });
      this->context_ptr = new rcl_context_t;
      *this->context_ptr = rcl_get_zero_initialized_context();
      ret = rcl_init(0, nullptr, &init_options, this->context_ptr);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    }
    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();
    const char * name = "test_message_fixture_node";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node_ptr, name, "", this->context_ptr, &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  void TearDown()
  {
    rcl_ret_t ret = rcl_node_fini(this->node_ptr);
    delete this->node_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_shutdown(this->context_ptr);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    ret = rcl_context_fini(this->context_ptr);
    delete this->context_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  template<typename MessageT>
  void test_message_type(const char * topic_name, const rosidl_message_type_support_t * ts)
  {
    rcl_ret_t ret;
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();

    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    ret = rcl_publisher_init(&publisher, this->node_ptr, ts, topic_name, &publisher_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    auto publisher_exit = make_scope_exit(
      [&publisher, this]() {
        rcl_ret_t ret = rcl_publisher_fini(&publisher, this->node_ptr);
        EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      });
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    ret = rcl_subscription_init(
      &subscription, this->node_ptr, ts, topic_name, &subscription_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    auto subscription_exit = make_scope_exit(
      [&subscription, this]() {
        rcl_ret_t ret = rcl_subscription_fini(&subscription, this->node_ptr);
        EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      });
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;

    {
      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      ret = rcl_wait_set_init(
        &wait_set,
        0,  // number_of_subscriptions
        1,  // number_of_guard_conditions
        0,  // number_of_timers
        0,  // number_of_clients
        0,  // number_of_services
        rcl_get_default_allocator());
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      ret = rcl_wait_set_clear(&wait_set);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      const rcl_guard_condition_t * graph_guard_condition =
        rcl_node_get_graph_guard_condition(this->node_ptr);
      ret = rcl_wait_set_add_guard_condition(
        &wait_set, graph_guard_condition, NULL);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      ret = rcl_wait(&wait_set, -1);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
      MessageT message;
      size_t nb_msgs = get_message_num(&message);
      for (size_t msg_cnt = 0; msg_cnt < nb_msgs; msg_cnt++) {
        get_message(&message, msg_cnt);
        auto msg_exit = make_scope_exit(
          [&message]() {
            fini_message(&message);
          });
        ret = rcl_publish(&publisher, &message);
        ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
      }
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
      MessageT message;
      size_t nb_msgs = get_message_num(&message);
      for (size_t msg_cnt = 0; msg_cnt < nb_msgs; msg_cnt++) {
        init_message(&message);
        auto msg_exit = make_scope_exit(
          [&message]() {
            fini_message(&message);
          });

        rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
        EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
        ret = rcl_wait_set_init(
          &wait_set,
          1,  // number_of_subscriptions
          0,  // number_of_guard_conditions
          0,  // number_of_timers
          0,  // number_of_clients
          0,  // number_of_services
          rcl_get_default_allocator());
        EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
        ret = rcl_wait_set_clear(&wait_set);
        EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
        ret = rcl_wait_set_add_subscription(&wait_set, &subscription, NULL);
        EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
        ret = rcl_wait(&wait_set, -1);
        ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
        ret = rcl_take(&subscription, &message, nullptr);
        ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
        verify_message(message, msg_cnt);
      }
    }
  }
};


// Utilities for test fixtures
template<typename MessageT>
size_t get_message_num(MessageT * msg);

template<typename MessageT>
void init_message(MessageT * msg);

template<typename MessageT>
void get_message(MessageT * msg, size_t msg_num);

template<typename MessageT>
void verify_message(MessageT & msg, size_t msg_num);

template<typename MessageT>
void fini_message(MessageT * msg);

#define DEFINE_FINI_MESSAGE(TYPE) \
  template<> \
  void fini_message(TYPE * msg) { \
    TYPE ## __fini(msg); \
  }

// Test publish/subscribe with a variety of messages

// Define functions and test cases for each message type
template<>
size_t get_message_num(test_msgs__msg__Primitives * msg)
{
  (void)msg;
  return 4;
}

template<>
void init_message(test_msgs__msg__Primitives * msg)
{
  test_msgs__msg__Primitives__init(msg);
}

template<>
void get_message(test_msgs__msg__Primitives * msg, size_t msg_num)
{
  test_msgs__msg__Primitives__init(msg);
  switch (msg_num) {
    case 0:
      msg->bool_value = false;
      msg->byte_value = 0;
      msg->char_value = '\0';
      msg->float32_value = 0.0f;
      msg->float64_value = 0;
      msg->int8_value = 0;
      msg->uint8_value = 0;
      msg->int16_value = 0;
      msg->uint16_value = 0;
      msg->int32_value = 0;
      msg->uint32_value = 0;
      msg->int64_value = 0;
      msg->uint64_value = 0;
      rosidl_generator_c__String__assign(&msg->string_value, "");
      break;
    case 1:
      msg->bool_value = true;
      msg->byte_value = 255;
      msg->char_value = '\x7f';
      msg->float32_value = 1.125f;
      msg->float64_value = 1.125;
      msg->int8_value = (std::numeric_limits<int8_t>::max)();
      msg->uint8_value = (std::numeric_limits<uint8_t>::max)();
      msg->int16_value = (std::numeric_limits<int16_t>::max)();
      msg->uint16_value = (std::numeric_limits<uint16_t>::max)();
      msg->int32_value = (std::numeric_limits<int32_t>::max)();
      msg->uint32_value = (std::numeric_limits<uint32_t>::max)();
      msg->int64_value = (std::numeric_limits<int64_t>::max)();
      msg->uint64_value = (std::numeric_limits<uint64_t>::max)();
      rosidl_generator_c__String__assign(&msg->string_value, "max value");
      break;
    case 2:
      msg->bool_value = false;
      msg->byte_value = 0;
      msg->char_value = 0x0;
      msg->float32_value = -2.125f;
      msg->float64_value = -2.125;
      msg->int8_value = (std::numeric_limits<int8_t>::min)();
      msg->uint8_value = 0;
      msg->int16_value = (std::numeric_limits<int16_t>::min)();
      msg->uint16_value = 0;
      msg->int32_value = (std::numeric_limits<int32_t>::min)();
      msg->uint32_value = 0;
      msg->int64_value = (std::numeric_limits<int64_t>::min)();
      msg->uint64_value = 0;
      rosidl_generator_c__String__assign(&msg->string_value, "min value");
      break;
    case 3:
      msg->bool_value = true;
      msg->byte_value = 1;
      msg->char_value = '\1';
      msg->float32_value = 1.0f;
      msg->float64_value = 1;
      msg->int8_value = 1;
      msg->uint8_value = 1;
      msg->int16_value = 1;
      msg->uint16_value = 1;
      msg->int32_value = 1;
      msg->uint32_value = 1;
      msg->int64_value = 1;
      msg->uint64_value = 1;
      char string_value[20000] = {};
      for (uint32_t i = 0; i < 20000; i++) {
        string_value[i] = '0' + (i % 10);
      }
      rosidl_generator_c__String__assignn(&msg->string_value, string_value, sizeof(string_value));
      break;
  }
}

template<>
void verify_message(test_msgs__msg__Primitives & message, size_t msg_num)
{
  test_msgs__msg__Primitives expected_msg;
  get_message(&expected_msg, msg_num);
  EXPECT_EQ(expected_msg.bool_value, message.bool_value);
  EXPECT_EQ(expected_msg.byte_value, message.byte_value);
  EXPECT_EQ(expected_msg.char_value, message.char_value);
  EXPECT_FLOAT_EQ(expected_msg.float32_value, message.float32_value);
  EXPECT_DOUBLE_EQ(expected_msg.float64_value, message.float64_value);
  EXPECT_EQ(expected_msg.int8_value, message.int8_value);
  EXPECT_EQ(expected_msg.uint8_value, message.uint8_value);
  EXPECT_EQ(expected_msg.int16_value, message.int16_value);
  EXPECT_EQ(expected_msg.uint16_value, message.uint16_value);
  EXPECT_EQ(expected_msg.int32_value, message.int32_value);
  EXPECT_EQ(expected_msg.uint32_value, message.uint32_value);
  EXPECT_EQ(expected_msg.int64_value, message.int64_value);
  EXPECT_EQ(expected_msg.uint64_value, message.uint64_value);
  EXPECT_EQ(0, strcmp(expected_msg.string_value.data, message.string_value.data));
}

DEFINE_FINI_MESSAGE(test_msgs__msg__Primitives)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_primitives) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, Primitives);
  test_message_type<test_msgs__msg__Primitives>("test_primitives", ts);
}

template<>
size_t get_message_num(test_msgs__msg__Nested * msg)
{
  (void)msg;
  return 4;
}

template<>
void init_message(test_msgs__msg__Nested * msg)
{
  test_msgs__msg__Nested__init(msg);
}

template<>
void get_message(test_msgs__msg__Nested * msg, size_t msg_num)
{
  test_msgs__msg__Nested__init(msg);
  get_message(&msg->primitive_values, msg_num);
}

template<>
void verify_message(test_msgs__msg__Nested & message, size_t msg_num)
{
  verify_message(message.primitive_values, msg_num);
}

DEFINE_FINI_MESSAGE(test_msgs__msg__Nested)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_nested) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, Nested);
  test_message_type<test_msgs__msg__Nested>("test_nested", ts);
}


template<>
size_t get_message_num(test_msgs__msg__Builtins * msg)
{
  (void)msg;
  return 1;
}

template<>
void init_message(test_msgs__msg__Builtins * msg)
{
  test_msgs__msg__Builtins__init(msg);
}

template<>
void get_message(test_msgs__msg__Builtins * msg, size_t msg_num)
{
  test_msgs__msg__Builtins__init(msg);
  if (msg_num == 0) {
    msg->duration_value.sec = -1234567890;
    msg->duration_value.nanosec = 123456789;
    msg->time_value.sec = -1234567890;
    msg->time_value.nanosec = 987654321;
  }
}

template<>
void verify_message(test_msgs__msg__Builtins & message, size_t msg_num)
{
  test_msgs__msg__Builtins expected_msg;
  get_message(&expected_msg, msg_num);
  EXPECT_EQ(expected_msg.duration_value.sec, message.duration_value.sec);
  EXPECT_EQ(expected_msg.duration_value.nanosec, message.duration_value.nanosec);
  EXPECT_EQ(expected_msg.time_value.sec, message.time_value.sec);
  EXPECT_EQ(expected_msg.time_value.nanosec, message.time_value.nanosec);
}

DEFINE_FINI_MESSAGE(test_msgs__msg__Builtins)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_builtins) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, Builtins);
  test_message_type<test_msgs__msg__Builtins>("test_builtins", ts);
}


template<>
size_t get_message_num(test_msgs__msg__StaticArrayPrimitives * msg)
{
  (void)msg;
  return 1;
}

template<>
void init_message(test_msgs__msg__StaticArrayPrimitives * msg)
{
  test_msgs__msg__StaticArrayPrimitives__init(msg);
}

template<>
void get_message(test_msgs__msg__StaticArrayPrimitives * msg, size_t msg_num)
{
  test_msgs__msg__StaticArrayPrimitives__init(msg);
  if (msg_num == 0) {
    msg->bool_values[0] = false;
    msg->bool_values[1] = true;
    msg->bool_values[2] = false;
    msg->byte_values[0] = 0;
    msg->byte_values[1] = 0xff;
    msg->byte_values[2] = 0;
    msg->char_values[0] = '\0';
    msg->char_values[1] = '\x7f';
    msg->char_values[2] = '\0';
    msg->float32_values[0] = 0.0f;
    msg->float32_values[1] = 1.125f;
    msg->float32_values[2] = -2.125f;
    msg->float64_values[0] = 0;
    msg->float64_values[1] = 1.125;
    msg->float64_values[2] = -2.125;
    msg->int8_values[0] = 0;
    msg->int8_values[1] = (std::numeric_limits<int8_t>::max)();
    msg->int8_values[2] = (std::numeric_limits<int8_t>::min)();
    msg->uint8_values[0] = 0;
    msg->uint8_values[1] = (std::numeric_limits<uint8_t>::max)();
    msg->uint8_values[2] = 0;
    msg->int16_values[0] = 0;
    msg->int16_values[1] = (std::numeric_limits<int16_t>::max)();
    msg->int16_values[2] = (std::numeric_limits<int16_t>::min)();
    msg->uint16_values[0] = 0;
    msg->uint16_values[1] = (std::numeric_limits<uint16_t>::max)();
    msg->uint16_values[2] = 0;
    msg->int32_values[0] = static_cast<int32_t>(0);
    msg->int32_values[1] = (std::numeric_limits<int32_t>::max)();
    msg->int32_values[2] = (std::numeric_limits<int32_t>::min)();
    msg->uint32_values[0] = 0;
    msg->uint32_values[1] = (std::numeric_limits<uint32_t>::max)();
    msg->uint32_values[2] = 0;
    msg->int64_values[0] = 0;
    msg->int64_values[1] = (std::numeric_limits<int64_t>::max)();
    msg->int64_values[2] = (std::numeric_limits<int64_t>::min)();
    msg->uint64_values[0] = 0;
    msg->uint64_values[1] = (std::numeric_limits<uint64_t>::max)();
    msg->uint64_values[2] = 0;
    rosidl_generator_c__String__assign(&msg->string_values[0], "");
    rosidl_generator_c__String__assign(&msg->string_values[1], "max value");
    rosidl_generator_c__String__assign(&msg->string_values[2], "min value");
  }
}

template<>
void verify_message(test_msgs__msg__StaticArrayPrimitives & message, size_t msg_num)
{
  test_msgs__msg__StaticArrayPrimitives expected_msg;
  get_message(&expected_msg, msg_num);
  for (size_t i = 0; i < 3; ++i) {
    EXPECT_EQ(expected_msg.bool_values[i], message.bool_values[i]);
    EXPECT_EQ(expected_msg.byte_values[i], message.byte_values[i]);
    EXPECT_EQ(expected_msg.char_values[i], message.char_values[i]);
    EXPECT_FLOAT_EQ(expected_msg.float32_values[i], message.float32_values[i]);
    EXPECT_DOUBLE_EQ(expected_msg.float64_values[i], message.float64_values[i]);
    EXPECT_EQ(expected_msg.int8_values[i], message.int8_values[i]);
    EXPECT_EQ(expected_msg.uint8_values[i], message.uint8_values[i]);
    EXPECT_EQ(expected_msg.int16_values[i], message.int16_values[i]);
    EXPECT_EQ(expected_msg.uint16_values[i], message.uint16_values[i]);
    EXPECT_EQ(expected_msg.int32_values[i], message.int32_values[i]);
    EXPECT_EQ(expected_msg.uint32_values[i], message.uint32_values[i]);
    EXPECT_EQ(expected_msg.int64_values[i], message.int64_values[i]);
    EXPECT_EQ(expected_msg.uint64_values[i], message.uint64_values[i]);
    EXPECT_EQ(0, strcmp(expected_msg.string_values[i].data,
      message.string_values[i].data));
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__StaticArrayPrimitives)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_staticarrayprimitives) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, StaticArrayPrimitives);
  test_message_type<test_msgs__msg__StaticArrayPrimitives>("test_staticarrayprimitives",
    ts);
}

template<>
size_t get_message_num(test_msgs__msg__StaticArrayNested * msg)
{
  (void)msg;
  return 1;
}

template<>
void init_message(test_msgs__msg__StaticArrayNested * msg)
{
  test_msgs__msg__StaticArrayNested__init(msg);
}

template<>
void get_message(test_msgs__msg__StaticArrayNested * msg, size_t msg_num)
{
  test_msgs__msg__StaticArrayNested__init(msg);
  test_msgs__msg__Primitives submsg;
  if (msg_num == 0) {
    for (size_t i = 0; i < get_message_num(&submsg); ++i) {
      get_message(&msg->primitive_values[i], i);
    }
  }
}

template<>
void verify_message(test_msgs__msg__StaticArrayNested & message, size_t msg_num)
{
  (void)msg_num;
  for (size_t i = 0; i < 4; ++i) {
    verify_message(message.primitive_values[i], i);
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__StaticArrayNested)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_staticarraynested) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, StaticArrayNested);
  test_message_type<test_msgs__msg__StaticArrayNested>("test_staticarraynested",
    ts);
}

template<>
size_t get_message_num(test_msgs__msg__DynamicArrayPrimitives * msg)
{
  (void)msg;
  return 5;
}

template<>
void init_message(test_msgs__msg__DynamicArrayPrimitives * msg)
{
  test_msgs__msg__DynamicArrayPrimitives__init(msg);
}

template<>
void get_message(test_msgs__msg__DynamicArrayPrimitives * msg, size_t msg_num)
{
  test_msgs__msg__DynamicArrayPrimitives__init(msg);
  const size_t size = 2000;
  switch (msg_num) {
    case 0:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, 0);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, 0);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, 0);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, 0);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, 0);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, 0);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, 0);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, 0);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, 0);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, 0);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, 0);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, 0);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, 0);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, 0);
      msg->check = 0;
      break;
    case 1:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, 1);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, 1);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, 1);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, 1);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, 1);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, 1);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, 1);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, 1);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, 1);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, 1);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, 1);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, 1);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, 1);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, 1);

      msg->bool_values.data[0] = true;
      msg->byte_values.data[0] = 0xff;
      msg->char_values.data[0] = '\x7f';
      msg->float32_values.data[0] = 1.125f;
      msg->float64_values.data[0] = 1.125;
      msg->int8_values.data[0] = (std::numeric_limits<int8_t>::max)();
      msg->uint8_values.data[0] = (std::numeric_limits<uint8_t>::max)();
      msg->int16_values.data[0] = (std::numeric_limits<int16_t>::max)();
      msg->uint16_values.data[0] = (std::numeric_limits<uint16_t>::max)();
      msg->int32_values.data[0] = (std::numeric_limits<int32_t>::max)();
      msg->uint32_values.data[0] = (std::numeric_limits<uint32_t>::max)();
      msg->int64_values.data[0] = (std::numeric_limits<int64_t>::max)();
      msg->uint64_values.data[0] = (std::numeric_limits<uint64_t>::max)();
      rosidl_generator_c__String__assign(&msg->string_values.data[0], "max value");
      msg->check = 1;
      break;
    case 2:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, 2);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, 2);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, 2);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, 3);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, 3);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, 3);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, 2);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, 3);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, 2);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, 3);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, 2);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, 3);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, 2);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, 3);

      msg->bool_values.data[0] = false;
      msg->bool_values.data[1] = true;
      msg->byte_values.data[0] = 0x00;
      msg->byte_values.data[1] = 0xff;
      msg->char_values.data[0] = '\0';
      msg->char_values.data[1] = '\x7f';
      msg->float32_values.data[0] = 0.0f;
      msg->float32_values.data[1] = 1.125f;
      msg->float32_values.data[2] = -2.125f;
      msg->float64_values.data[0] = 0;
      msg->float64_values.data[1] = 1.125;
      msg->float64_values.data[2] = -2.125;
      msg->int8_values.data[0] = 0;
      msg->int8_values.data[1] = (std::numeric_limits<int8_t>::max)();
      msg->int8_values.data[2] = (std::numeric_limits<int8_t>::min)();
      msg->uint8_values.data[0] = 0;
      msg->uint8_values.data[1] = (std::numeric_limits<uint8_t>::max)();
      msg->int16_values.data[0] = 0;
      msg->int16_values.data[1] = (std::numeric_limits<int16_t>::max)();
      msg->int16_values.data[2] = (std::numeric_limits<int16_t>::min)();
      msg->uint16_values.data[0] = 0;
      msg->uint16_values.data[1] = (std::numeric_limits<uint16_t>::max)();
      msg->int32_values.data[0] = 0;
      msg->int32_values.data[1] = (std::numeric_limits<int32_t>::max)();
      msg->int32_values.data[2] = (std::numeric_limits<int32_t>::min)();
      msg->uint32_values.data[0] = 0;
      msg->uint32_values.data[1] = (std::numeric_limits<uint32_t>::max)();
      msg->int64_values.data[0] = 0;
      msg->int64_values.data[1] = (std::numeric_limits<int64_t>::max)();
      msg->int64_values.data[2] = (std::numeric_limits<int64_t>::min)();
      msg->uint64_values.data[0] = 0;
      msg->uint64_values.data[1] = (std::numeric_limits<uint64_t>::max)();
      rosidl_generator_c__String__assign(&msg->string_values.data[0], "");
      rosidl_generator_c__String__assign(&msg->string_values.data[1], "max value");
      rosidl_generator_c__String__assign(&msg->string_values.data[2], "optional min value");
      msg->check = 2;
      break;
    case 3:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, size);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, size);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, size);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, size);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, size);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, size);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, size);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, size);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, size);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, size);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, size);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, size);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, size);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, size);

      for (size_t i = 0; i < size; ++i) {
        msg->bool_values.data[i] = (i % 2 != 0) ? true : false;
        msg->byte_values.data[i] = (uint8_t)i;
        msg->char_values.data[i] = static_cast<char>(i);
        msg->float32_values.data[i] = 1.125f * i;
        msg->float64_values.data[i] = 1.125 * i;
        msg->int8_values.data[i] = (int8_t)i;
        msg->uint8_values.data[i] = (uint8_t)i;
        msg->int16_values.data[i] = (int16_t)i;
        msg->uint16_values.data[i] = (uint16_t)i;
        msg->int32_values.data[i] = (int32_t)i;
        msg->uint32_values.data[i] = (uint32_t)i;
        msg->int64_values.data[i] = (int64_t)i;
        msg->uint64_values.data[i] = (uint64_t)i;
        // Here we use 21 to represent `size` as a string
        // we need 20 characters to represent all size_t values (assuming it's 64bits)
        // +1 character for the null-terminator
        char tmpstr[21];
        snprintf(tmpstr, sizeof(tmpstr), "%zu", i);
        rosidl_generator_c__String__assign(&msg->string_values.data[i], tmpstr);
      }
      msg->check = 3;
      break;
    case 4:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, 0);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, 0);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, 0);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, 0);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, 0);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, 0);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, 0);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, 0);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, 0);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, 0);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, 0);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, 0);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, 0);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, 0);
      msg->check = 4;
      break;
  }
}

template<>
void verify_message(test_msgs__msg__DynamicArrayPrimitives & message, size_t msg_num)
{
  test_msgs__msg__DynamicArrayPrimitives expected_msg;
  get_message(&expected_msg, msg_num);
  for (size_t i = 0; i < expected_msg.bool_values.size; ++i) {
    EXPECT_EQ(expected_msg.bool_values.data[i],
      message.bool_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.byte_values.size; ++i) {
    EXPECT_EQ(expected_msg.byte_values.data[i],
      message.byte_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.char_values.size; ++i) {
    EXPECT_EQ(expected_msg.char_values.data[i],
      message.char_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.float32_values.size; ++i) {
    EXPECT_FLOAT_EQ(expected_msg.float32_values.data[i],
      message.float32_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.float64_values.size; ++i) {
    EXPECT_DOUBLE_EQ(expected_msg.float64_values.data[i],
      message.float64_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int8_values.size; ++i) {
    EXPECT_EQ(expected_msg.int8_values.data[i],
      message.int8_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint8_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint8_values.data[i],
      message.uint8_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int16_values.size; ++i) {
    EXPECT_EQ(expected_msg.int16_values.data[i],
      message.int16_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint16_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint16_values.data[i],
      message.uint16_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int32_values.size; ++i) {
    EXPECT_EQ(expected_msg.int32_values.data[i],
      message.int32_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint32_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint32_values.data[i],
      message.uint32_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int64_values.size; ++i) {
    EXPECT_EQ(expected_msg.int64_values.data[i],
      message.int64_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint64_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint64_values.data[i],
      message.uint64_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.string_values.size; ++i) {
    EXPECT_EQ(0, strcmp(message.string_values.data[i].data,
      expected_msg.string_values.data[i].data));
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__DynamicArrayPrimitives)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_dynamicarrayprimitives) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, DynamicArrayPrimitives);
  test_message_type<test_msgs__msg__DynamicArrayPrimitives>("test_dynamicarrayprimitives",
    ts);
}

template<>
size_t get_message_num(test_msgs__msg__DynamicArrayNested * msg)
{
  (void)msg;
  return 1;
}

template<>
void init_message(test_msgs__msg__DynamicArrayNested * msg)
{
  test_msgs__msg__DynamicArrayNested__init(msg);
}

template<>
void get_message(test_msgs__msg__DynamicArrayNested * msg, size_t msg_num)
{
  test_msgs__msg__Primitives submsg;
  const size_t size = get_message_num(&submsg);
  test_msgs__msg__DynamicArrayNested__init(msg);
  test_msgs__msg__Primitives__Sequence__init(&msg->primitive_values, size);
  switch (msg_num) {
    case 0:
      for (size_t i = 0; i < size; ++i) {
        get_message(&msg->primitive_values.data[i], i);
      }
      break;
  }
}

template<>
void verify_message(test_msgs__msg__DynamicArrayNested & message, size_t msg_num)
{
  test_msgs__msg__DynamicArrayNested expected_msg;
  get_message(&expected_msg, msg_num);

  for (size_t i = 0; i < expected_msg.primitive_values.size; ++i) {
    EXPECT_EQ(expected_msg.primitive_values.data[i].bool_value,
      message.primitive_values.data[i].bool_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].byte_value,
      message.primitive_values.data[i].byte_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].char_value,
      message.primitive_values.data[i].char_value);
    EXPECT_FLOAT_EQ(expected_msg.primitive_values.data[i].float32_value,
      message.primitive_values.data[i].float32_value);
    EXPECT_DOUBLE_EQ(expected_msg.primitive_values.data[i].float64_value,
      message.primitive_values.data[i].float64_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int8_value,
      message.primitive_values.data[i].int8_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint8_value,
      message.primitive_values.data[i].uint8_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int16_value,
      message.primitive_values.data[i].int16_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint16_value,
      message.primitive_values.data[i].uint16_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int32_value,
      message.primitive_values.data[i].int32_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint32_value,
      message.primitive_values.data[i].uint32_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int64_value,
      message.primitive_values.data[i].int64_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint64_value,
      message.primitive_values.data[i].uint64_value);
    EXPECT_EQ(0, strcmp(message.primitive_values.data[i].string_value.data,
      expected_msg.primitive_values.data[i].string_value.data));
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__DynamicArrayNested)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_dynamicarraynested) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, DynamicArrayNested);
  test_message_type<test_msgs__msg__DynamicArrayNested>("test_dynamicarraynested",
    ts);
}


template<>
size_t get_message_num(test_msgs__msg__BoundedArrayPrimitives * msg)
{
  (void)msg;
  return 5;
}

template<>
void init_message(test_msgs__msg__BoundedArrayPrimitives * msg)
{
  test_msgs__msg__BoundedArrayPrimitives__init(msg);
}

template<>
void get_message(test_msgs__msg__BoundedArrayPrimitives * msg, size_t msg_num)
{
  test_msgs__msg__BoundedArrayPrimitives__init(msg);
  switch (msg_num) {
    case 0:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, 3);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, 3);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, 3);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, 3);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, 3);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, 3);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, 3);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, 3);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, 3);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, 3);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, 3);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, 3);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, 3);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, 3);

      msg->bool_values.data[0] = false;
      msg->bool_values.data[1] = true;
      msg->bool_values.data[2] = false;
      msg->byte_values.data[0] = 0x00;
      msg->byte_values.data[1] = 0x01;
      msg->byte_values.data[2] = 0xff;
      msg->char_values.data[0] = '\0';
      msg->char_values.data[1] = '\1';
      msg->char_values.data[2] = '\255';
      msg->float32_values.data[0] = 0.0f;
      msg->float32_values.data[1] = 1.125f;
      msg->float32_values.data[2] = -2.125f;
      msg->float64_values.data[0] = 0;
      msg->float64_values.data[1] = 1.125;
      msg->float64_values.data[2] = -2.125;
      msg->int8_values.data[0] = 0;
      msg->int8_values.data[1] = (std::numeric_limits<int8_t>::max)();
      msg->int8_values.data[2] = (std::numeric_limits<int8_t>::min)();
      msg->uint8_values.data[0] = 0;
      msg->uint8_values.data[1] = 1;
      msg->uint8_values.data[2] = (std::numeric_limits<uint8_t>::max)();
      msg->int16_values.data[0] = 0;
      msg->int16_values.data[1] = (std::numeric_limits<int16_t>::max)();
      msg->int16_values.data[2] = (std::numeric_limits<int16_t>::min)();
      msg->uint16_values.data[0] = 0;
      msg->uint16_values.data[1] = 1;
      msg->uint16_values.data[2] = (std::numeric_limits<uint16_t>::max)();
      msg->int32_values.data[0] = 0;
      msg->int32_values.data[1] = (std::numeric_limits<int32_t>::max)();
      msg->int32_values.data[2] = (std::numeric_limits<int32_t>::min)();
      msg->uint32_values.data[0] = 0;
      msg->uint32_values.data[1] = 1;
      msg->uint32_values.data[2] = (std::numeric_limits<uint32_t>::max)();
      msg->int64_values.data[0] = 0;
      msg->int64_values.data[1] = (std::numeric_limits<int64_t>::max)();
      msg->int64_values.data[2] = (std::numeric_limits<int64_t>::min)();
      msg->uint64_values.data[0] = 0;
      msg->uint64_values.data[1] = 1;
      msg->uint64_values.data[2] = (std::numeric_limits<uint64_t>::max)();
      rosidl_generator_c__String__assign(&msg->string_values.data[0], "");
      rosidl_generator_c__String__assign(&msg->string_values.data[1], "max value");
      rosidl_generator_c__String__assign(&msg->string_values.data[2], "optional min value");
      msg->check = 2;
      break;
    case 1:
      rosidl_generator_c__bool__Sequence__init(&msg->bool_values, 0);
      rosidl_generator_c__byte__Sequence__init(&msg->byte_values, 0);
      rosidl_generator_c__char__Sequence__init(&msg->char_values, 0);
      rosidl_generator_c__float32__Sequence__init(&msg->float32_values, 0);
      rosidl_generator_c__float64__Sequence__init(&msg->float64_values, 0);
      rosidl_generator_c__int8__Sequence__init(&msg->int8_values, 0);
      rosidl_generator_c__uint8__Sequence__init(&msg->uint8_values, 0);
      rosidl_generator_c__int16__Sequence__init(&msg->int16_values, 0);
      rosidl_generator_c__uint16__Sequence__init(&msg->uint16_values, 0);
      rosidl_generator_c__int32__Sequence__init(&msg->int32_values, 0);
      rosidl_generator_c__uint32__Sequence__init(&msg->uint32_values, 0);
      rosidl_generator_c__int64__Sequence__init(&msg->int64_values, 0);
      rosidl_generator_c__uint64__Sequence__init(&msg->uint64_values, 0);
      rosidl_generator_c__String__Sequence__init(&msg->string_values, 0);
      msg->check = 4;
      break;
  }
}

template<>
void verify_message(test_msgs__msg__BoundedArrayPrimitives & message, size_t msg_num)
{
  test_msgs__msg__BoundedArrayPrimitives expected_msg;
  get_message(&expected_msg, msg_num);
  for (size_t i = 0; i < expected_msg.bool_values.size; ++i) {
    EXPECT_EQ(expected_msg.bool_values.data[i],
      message.bool_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.byte_values.size; ++i) {
    EXPECT_EQ(expected_msg.byte_values.data[i],
      message.byte_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.char_values.size; ++i) {
    EXPECT_EQ(expected_msg.char_values.data[i],
      message.char_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.float32_values.size; ++i) {
    EXPECT_FLOAT_EQ(expected_msg.float32_values.data[i],
      message.float32_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.float64_values.size; ++i) {
    EXPECT_DOUBLE_EQ(expected_msg.float64_values.data[i],
      message.float64_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int8_values.size; ++i) {
    EXPECT_EQ(expected_msg.int8_values.data[i],
      message.int8_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint8_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint8_values.data[i],
      message.uint8_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int16_values.size; ++i) {
    EXPECT_EQ(expected_msg.int16_values.data[i],
      message.int16_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint16_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint16_values.data[i],
      message.uint16_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int32_values.size; ++i) {
    EXPECT_EQ(expected_msg.int32_values.data[i],
      message.int32_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint32_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint32_values.data[i],
      message.uint32_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.int64_values.size; ++i) {
    EXPECT_EQ(expected_msg.int64_values.data[i],
      message.int64_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.uint64_values.size; ++i) {
    EXPECT_EQ(expected_msg.uint64_values.data[i],
      message.uint64_values.data[i]);
  }
  for (size_t i = 0; i < expected_msg.string_values.size; ++i) {
    EXPECT_EQ(0, strcmp(message.string_values.data[i].data,
      expected_msg.string_values.data[i].data));
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__BoundedArrayPrimitives)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_boundedarrayprimitives) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, BoundedArrayPrimitives);
  test_message_type<test_msgs__msg__BoundedArrayPrimitives>("test_boundedarrayprimitives",
    ts);
}

template<>
size_t get_message_num(test_msgs__msg__BoundedArrayNested * msg)
{
  (void)msg;
  return 1;
}


template<>
void init_message(test_msgs__msg__BoundedArrayNested * msg)
{
  test_msgs__msg__BoundedArrayNested__init(msg);
}

template<>
void get_message(test_msgs__msg__BoundedArrayNested * msg, size_t msg_num)
{
  test_msgs__msg__Primitives submsg;
  const size_t size = get_message_num(&submsg);
  test_msgs__msg__BoundedArrayNested__init(msg);
  test_msgs__msg__Primitives__Sequence__init(&msg->primitive_values, size);
  switch (msg_num) {
    case 0:
      for (size_t i = 0; i < size; ++i) {
        get_message(&msg->primitive_values.data[i], i);
      }
      break;
  }
}

template<>
void verify_message(test_msgs__msg__BoundedArrayNested & message, size_t msg_num)
{
  test_msgs__msg__BoundedArrayNested expected_msg;
  get_message(&expected_msg, msg_num);

  for (size_t i = 0; i < expected_msg.primitive_values.size; ++i) {
    EXPECT_EQ(expected_msg.primitive_values.data[i].bool_value,
      message.primitive_values.data[i].bool_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].byte_value,
      message.primitive_values.data[i].byte_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].char_value,
      message.primitive_values.data[i].char_value);
    EXPECT_FLOAT_EQ(expected_msg.primitive_values.data[i].float32_value,
      message.primitive_values.data[i].float32_value);
    EXPECT_DOUBLE_EQ(expected_msg.primitive_values.data[i].float64_value,
      message.primitive_values.data[i].float64_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int8_value,
      message.primitive_values.data[i].int8_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint8_value,
      message.primitive_values.data[i].uint8_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int16_value,
      message.primitive_values.data[i].int16_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint16_value,
      message.primitive_values.data[i].uint16_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int32_value,
      message.primitive_values.data[i].int32_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint32_value,
      message.primitive_values.data[i].uint32_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].int64_value,
      message.primitive_values.data[i].int64_value);
    EXPECT_EQ(expected_msg.primitive_values.data[i].uint64_value,
      message.primitive_values.data[i].uint64_value);
    EXPECT_EQ(0, strcmp(message.primitive_values.data[i].string_value.data,
      expected_msg.primitive_values.data[i].string_value.data));
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__BoundedArrayNested)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_boundedarraynested) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, BoundedArrayNested);
  test_message_type<test_msgs__msg__BoundedArrayNested>("test_boundedarraynested",
    ts);
}

template<>
size_t get_message_num(test_msgs__msg__DynamicArrayPrimitivesNested * msg)
{
  (void)msg;
  return 1;
}


template<>
void init_message(test_msgs__msg__DynamicArrayPrimitivesNested * msg)
{
  test_msgs__msg__DynamicArrayPrimitivesNested__init(msg);
}

template<>
void get_message(test_msgs__msg__DynamicArrayPrimitivesNested * msg, size_t msg_num)
{
  test_msgs__msg__DynamicArrayPrimitives submsg;
  const size_t size = get_message_num(&submsg);
  test_msgs__msg__DynamicArrayPrimitivesNested__init(msg);
  test_msgs__msg__DynamicArrayPrimitives__Sequence__init(
    &msg->dynamic_array_primitive_values, size);
  switch (msg_num) {
    case 0:
      for (size_t i = 0; i < size; ++i) {
        get_message(&msg->dynamic_array_primitive_values.data[i], i);
      }
      break;
  }
}

template<>
void verify_message(test_msgs__msg__DynamicArrayPrimitivesNested & message, size_t msg_num)
{
  test_msgs__msg__DynamicArrayPrimitivesNested expected_msg;
  get_message(&expected_msg, msg_num);
  for (size_t i = 0; i < expected_msg.dynamic_array_primitive_values.size; ++i) {
    verify_message(message.dynamic_array_primitive_values.data[i], i);
  }
}

DEFINE_FINI_MESSAGE(test_msgs__msg__DynamicArrayPrimitivesNested)
TEST_F(CLASSNAME(TestMessagesFixture, RMW_IMPLEMENTATION), test_dynamicarraynestedprimitives) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_msgs, msg, DynamicArrayPrimitivesNested);
  test_message_type<test_msgs__msg__DynamicArrayPrimitivesNested>(
    "test_dynamicarraynestedprimitives", ts);
}
