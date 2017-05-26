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

void custom_putenv(const char * name, const char * value)
{
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
    ret = rcl_shutdown();
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
  }

  void test_node_creation(
    const char * ROS_SECURE_ROOT, const char * ROS_ENABLE_SECURITY,
    const char * ROS_SECURITY_STRATEGY,
    const char * node_name, bool should_fail_participant_creation)
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
    if (should_fail_participant_creation) {
      ASSERT_EQ(RCL_RET_ERROR, ret) << rcl_get_error_string_safe();
    } else {
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string_safe();
    }
  }
};

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_1) {
  test_node_creation(
    "garbage",
    "true", "Enforce",
    "publisher", true);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_2) {
  test_node_creation(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files",
    "true", "Enforce",
    "publisher2", true);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_3) {
  test_node_creation(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files",
    "true", "Enforce",
    "publisher_invalid_cert", true);
}

TEST_F(CLASSNAME(TestSecureNode, RMW_IMPLEMENTATION), test_4) {
  test_node_creation(
    "/home/mikael/work/ros2/sros2_testing_ws/src/ros2/system_tests/test_communication/test/test_security_files",
    "true", "Garbage",
    "publisher_missing_key", true);
}
