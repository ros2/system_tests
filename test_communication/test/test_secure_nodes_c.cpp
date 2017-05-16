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

#include "rcl/subscription.h"
#include "rcl/publisher.h"

#include "rcl/rcl.h"

// #include "test_communication/msg/bounded_array_nested.h"
// #include "test_communication/msg/bounded_array_primitives.h"
// #include "test_communication/msg/dynamic_array_nested.h"
// #include "test_communication/msg/dynamic_array_primitives.h"
#include "test_communication/msg/empty.h"
#include "test_communication/msg/nested.h"
#include "test_communication/msg/primitives.h"
// #include "test_communication/msg/static_array_nested.h"
// #include "test_communication/msg/static_array_primitives.h"
// #include "test_communication/msg/builtins.h"

#include "rosidl_generator_c/string_functions.h"
#include "rosidl_generator_c/primitives_array_functions.h"
#include "rosidl_generator_c/message_type_support_struct.h"
#include "rcl/error_handling.h"
#include "rcutils/get_env.h"
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

void custom_putenv(const char * name, const char * value) {
#ifdef WIN32
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif
}

#define SCOPE_EXIT(code) make_scope_exit([&]() {code; })

#endif  // SCOPE_EXIT_HPP_


#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestSecureNode, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  rcl_node_t * node_ptr;
  void SetUp()
  {
  }

  void TearDown()
  {
    rcl_ret_t ret = rcl_node_fini(this->node_ptr);
    delete this->node_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_shutdown();
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  }

  void test_node_connection(
    const char * ROS_SECURE_ROOT, const char * ROS_ENABLE_SECURITY, const char * ROS_SECURITY_STRATEGY,
    const char * node_name, const char * topic_name, const rosidl_message_type_support_t * ts)
  {
    custom_putenv("ROS_SECURE_ROOT", ROS_SECURE_ROOT);
    custom_putenv("ROS_ENABLE_SECURITY", ROS_ENABLE_SECURITY);
    custom_putenv("ROS_SECURITY_STRATEGY", ROS_SECURITY_STRATEGY);
    rcl_ret_t ret;
    ret = rcl_init(0, nullptr, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();
    // const char * name = "node_name";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node_ptr, node_name, "", &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();

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
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    EXPECT_EQ(0, strcmp(rcl_subscription_get_topic_name(&subscription), topic_name));

    // this waitset will return if an event happened, in this case,
    // the graph guard_condition notifying
    // that the publisher and the subscription have been matched
    {
      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_init(
        &wait_set,
        0,  // number_of_subscriptions
        1,  // number_of_guard_conditions
        0,  // number_of_timers
        0,  // number_of_clients
        0,  // number_of_services
        rcl_get_default_allocator());
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_clear_guard_conditions(&wait_set);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      const rcl_guard_condition_t * graph_guard_condition =
        rcl_node_get_graph_guard_condition(this->node_ptr);
      ret = rcl_wait_set_add_guard_condition(
        &wait_set, graph_guard_condition);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait(&wait_set, -1);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
      test_communication__msg__Primitives msg;
      test_communication__msg__Primitives__init(&msg);
      msg.bool_value = true;
      msg.byte_value = 0;
      msg.char_value = '\0';
      msg.float32_value = 0.0f;
      msg.float64_value = 0;
      msg.int8_value = 0;
      msg.uint8_value = 0;
      msg.int16_value = 0;
      msg.uint16_value = 0;
      msg.int32_value = 0;
      msg.uint32_value = 0;
      msg.int64_value = 0;
      msg.uint64_value = 0;
      rosidl_generator_c__String__assign(&msg.string_value, "coucou");

      auto msg_exit = make_scope_exit([&msg]() {
        test_communication__msg__Primitives__fini(&msg);
      });
      ret = rcl_publish(&publisher, &msg);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
      test_communication__msg__Primitives msg;
      test_communication__msg__Primitives__init(&msg);

      auto msg_exit = make_scope_exit([&msg]() {
        test_communication__msg__Primitives__fini(&msg);
      });

      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_init(
        &wait_set,
        1,  // number_of_subscriptions
        0,  // number_of_guard_conditions
        0,  // number_of_timers
        0,  // number_of_clients
        0,  // number_of_services
        rcl_get_default_allocator());
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_clear_subscriptions(&wait_set);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_add_subscription(&wait_set, &subscription);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait(&wait_set, -1);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_take(&subscription, &msg, nullptr);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }
  }
};

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_chatter_disabled_security) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_node_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "false", "Enforce",
    "publisher", "chatter", ts);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_chatter_disabled_security2) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_node_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "false", "Garbage",
    "publisher", "chatter", ts);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_chatter_disabled_security3) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_node_connection(
    "Garbage", "false", "Garbage",
    "publisher", "chatter", ts);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_chatter_permissive_security) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_node_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Permissive",
    "publisher", "chatter", ts);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_chatter_permissive_security_certificated_not_found) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_node_connection(
    "Garbage", "true", "Permissive",
    "publisher", "chatter", ts);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_chatter_enforced_security) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_node_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Enforce",
    "publisher", "chatter", ts);
}

class CLASSNAME (TestSecureNodes, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  rcl_node_t * node1_ptr;
  rcl_node_t * node2_ptr;
  void SetUp()
  {
  }

  void TearDown()
  {
    rcl_ret_t ret = rcl_node_fini(this->node1_ptr);
    delete this->node1_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_node_fini(this->node2_ptr);
    delete this->node2_ptr;
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    ret = rcl_shutdown();
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  }

  void test_nodes_connection(
    const char * node1_ROS_SECURE_ROOT, const char * node1_ROS_ENABLE_SECURITY, const char * node1_ROS_SECURITY_STRATEGY,
    const char * node1_name, const char * node1_topic_name,
    const char * node2_ROS_SECURE_ROOT, const char * node2_ROS_ENABLE_SECURITY, const char * node2_ROS_SECURITY_STRATEGY,
    const char * node2_name, const char * node2_topic_name,
    bool should_connect,
    const rosidl_message_type_support_t * ts)
  {
    custom_putenv("ROS_SECURE_ROOT", node1_ROS_SECURE_ROOT);
    custom_putenv("ROS_ENABLE_SECURITY", node1_ROS_ENABLE_SECURITY);
    custom_putenv("ROS_SECURITY_STRATEGY", node1_ROS_SECURITY_STRATEGY);

    const char * env_val = nullptr;
    ASSERT_FALSE(rcutils_get_env("ROS_SECURITY_STRATEGY", &env_val));
    ASSERT_STREQ(env_val, node1_ROS_SECURITY_STRATEGY);
    rcl_ret_t ret;
    ret = rcl_init(0, nullptr, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    this->node1_ptr = new rcl_node_t;
    *this->node1_ptr = rcl_get_zero_initialized_node();
    rcl_node_options_t node1_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node1_ptr, node1_name, "", &node1_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();

    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();

    rcl_publisher_options_t publisher_options = rcl_publisher_get_default_options();
    ret = rcl_publisher_init(&publisher, this->node1_ptr, ts, node1_topic_name, &publisher_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    auto publisher_exit = make_scope_exit([&publisher, this]() {
      rcl_ret_t ret = rcl_publisher_fini(&publisher, this->node1_ptr);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    });


    custom_putenv("ROS_SECURE_ROOT", node2_ROS_SECURE_ROOT);
    custom_putenv("ROS_ENABLE_SECURITY", node2_ROS_ENABLE_SECURITY);
    custom_putenv("ROS_SECURITY_STRATEGY", node2_ROS_SECURITY_STRATEGY);

    const char * env_val2 = nullptr;
    ASSERT_FALSE(rcutils_get_env("ROS_SECURITY_STRATEGY", &env_val2));
    ASSERT_STREQ(env_val2, node2_ROS_SECURITY_STRATEGY);
    this->node2_ptr = new rcl_node_t;
    *this->node2_ptr = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node2_ptr, node2_name, "", &node_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
    ret = rcl_subscription_init(
      &subscription, this->node2_ptr, ts, node2_topic_name, &subscription_options);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    auto subscription_exit = make_scope_exit([&subscription, this]() {
      rcl_ret_t ret = rcl_subscription_fini(&subscription, this->node2_ptr);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    });
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    EXPECT_EQ(0, strcmp(rcl_subscription_get_topic_name(&subscription), node2_topic_name));

    // this waitset will return if an event happened, in this case,
    // the graph guard_condition notifying
    // that the publisher and the subscription have been matched
    {
      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_init(
        &wait_set,
        0,  // number_of_subscriptions
        1,  // number_of_guard_conditions
        0,  // number_of_timers
        0,  // number_of_clients
        0,  // number_of_services
        rcl_get_default_allocator());
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_clear_guard_conditions(&wait_set);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      const rcl_guard_condition_t * graph_guard_condition =
        rcl_node_get_graph_guard_condition(this->node1_ptr);
      ret = rcl_wait_set_add_guard_condition(
        &wait_set, graph_guard_condition);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait(&wait_set, -1);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
      test_communication__msg__Primitives msg;
      test_communication__msg__Primitives__init(&msg);
      msg.bool_value = true;
      msg.byte_value = 0;
      msg.char_value = '\0';
      msg.float32_value = 0.0f;
      msg.float64_value = 0;
      msg.int8_value = 0;
      msg.uint8_value = 0;
      msg.int16_value = 0;
      msg.uint16_value = 0;
      msg.int32_value = 0;
      msg.uint32_value = 0;
      msg.int64_value = 0;
      msg.uint64_value = 0;
      rosidl_generator_c__String__assign(&msg.string_value, "coucou");

      auto msg_exit = make_scope_exit([&msg]() {
        test_communication__msg__Primitives__fini(&msg);
      });
      ret = rcl_publish(&publisher, &msg);
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    {
      test_communication__msg__Primitives msg;
      test_communication__msg__Primitives__init(&msg);

      auto msg_exit = make_scope_exit([&msg]() {
        test_communication__msg__Primitives__fini(&msg);
      });

      rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_init(
        &wait_set,
        1,  // number_of_subscriptions
        0,  // number_of_guard_conditions
        0,  // number_of_timers
        0,  // number_of_clients
        0,  // number_of_services
        rcl_get_default_allocator());
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_clear_subscriptions(&wait_set);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait_set_add_subscription(&wait_set, &subscription);
      EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      ret = rcl_wait(&wait_set, 5000000000);  // RCL_S_TO_NS((int64_t)10));
      if (should_connect) {
        ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
        ret = rcl_take(&subscription, &msg, nullptr);
        ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
      } else {
        ASSERT_EQ(RCL_RET_TIMEOUT, ret) << rcl_get_error_string_safe();
      }
    }
  }
};

TEST_F(CLASSNAME(TestSecureNodes, RMW_IMPLEMENTATION), test_pub_sub_permissive) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_nodes_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Permissive",
    "publisher2", "chatter",
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Permissive",
    "subscriber", "chatter", 
    true, ts);
}

TEST_F(CLASSNAME(TestSecureNodes, RMW_IMPLEMENTATION), test_pub_sub_enforce_permissive) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_nodes_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Enforce",
    "publisher", "chatter",
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Permissive",
    "subscriber", "chatter", 
    true, ts);
}

TEST_F(CLASSNAME(TestSecureNodes, RMW_IMPLEMENTATION), test_pub_sub_permissive_enforce) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_nodes_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Permissive",
    "publisher", "chatter",
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Enforce",
    "subscriber", "chatter", 
    true, ts);
}

TEST_F(CLASSNAME(TestSecureNodes, RMW_IMPLEMENTATION), test_pub_sub_permissive2) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_nodes_connection(
    "Garbage", "false", "WHATEVER",
    "publisher", "chatter",
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Permissive",
    "subscriber", "chatter", 
    true, ts);
}

TEST_F(CLASSNAME(TestSecureNodes, RMW_IMPLEMENTATION), test_pub_sub_enforce_fail_to_connect) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Primitives);
  test_nodes_connection(
    "Garbage", "false", "WHATEVER",
    "publisher", "chatter",
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "true", "Enforce",
    "subscriber", "chatter", 
    false, ts);
}

TEST_F(CLASSNAME(TestSecureNodes, RMW_IMPLEMENTATION), test_pub_sub_security_disabled) {
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(
    test_communication, msg, Empty);
  test_nodes_connection(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "false", "Permissive",
    "publisher", "chatter2",
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files", "false", "Enforce",
    "subscriber", "chatter", 
    true, ts);
}
