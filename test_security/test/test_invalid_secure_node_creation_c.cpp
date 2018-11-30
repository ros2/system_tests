// Copyright 2016-2017 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rcutils/get_env.h"

#ifndef SCOPE_EXIT_HPP_
#define SCOPE_EXIT_HPP_

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

void custom_putenv(const char * name, const char * value)
{
#ifdef _WIN32
  _putenv_s(name, value);
#else
  setenv(name, value, 1);
#endif  // _WIN32
}

#define SCOPE_EXIT(code) make_scope_exit([&]() {code;})

#endif  // SCOPE_EXIT_HPP_


#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

class CLASSNAME (TestInvalidSecureNode, RMW_IMPLEMENTATION) : public ::testing::Test
{
public:
  rcl_context_t context;
  rcl_node_t * node_ptr;
  void SetUp()
  {
  }

  void TearDown()
  {
    rcl_ret_t ret = rcl_node_fini(this->node_ptr);
    delete this->node_ptr;
    ret = rcl_shutdown(&context);
    EXPECT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
  }

  void test_node_creation(
    const char * ROS_SECURITY_ROOT_DIRECTORY, const char * ROS_SECURITY_ENABLE,
    const char * ROS_SECURITY_STRATEGY,
    const char * node_name, bool should_fail_participant_creation)
  {
    if (ROS_SECURITY_ROOT_DIRECTORY != NULL) {
      custom_putenv("ROS_SECURITY_ROOT_DIRECTORY", ROS_SECURITY_ROOT_DIRECTORY);
    }
    custom_putenv("ROS_SECURITY_ENABLE", ROS_SECURITY_ENABLE);
    custom_putenv("ROS_SECURITY_STRATEGY", ROS_SECURITY_STRATEGY);
    rcl_ret_t ret;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    context = rcl_get_zero_initialized_context();
    ret = rcl_init(0, nullptr, &init_options, &context);
    ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    this->node_ptr = new rcl_node_t;
    *this->node_ptr = rcl_get_zero_initialized_node();
    // const char * name = "node_name";
    rcl_node_options_t node_options = rcl_node_get_default_options();
    ret = rcl_node_init(this->node_ptr, node_name, "", &context, &node_options);
    if (should_fail_participant_creation) {
      ASSERT_EQ(RCL_RET_ERROR, ret) << rcl_get_error_string().str;
    } else {
      ASSERT_EQ(RCL_RET_OK, ret) << rcl_get_error_string().str;
    }
  }
};

TEST_F(CLASSNAME(TestInvalidSecureNode, RMW_IMPLEMENTATION), test_1) {
  test_node_creation(
    "garbage",
    "true", "Enforce",
    "publisher", true);
}

TEST_F(CLASSNAME(TestInvalidSecureNode, RMW_IMPLEMENTATION), test_2) {
  test_node_creation(
    NULL,
    "true", "Enforce",
    "publisher2", true);
}

TEST_F(CLASSNAME(TestInvalidSecureNode, RMW_IMPLEMENTATION), test_3) {
  test_node_creation(
    NULL,
    "true", "Enforce",
    "publisher_invalid_cert", true);
}

TEST_F(CLASSNAME(TestInvalidSecureNode, RMW_IMPLEMENTATION), test_4) {
  test_node_creation(
    NULL,
    "true", "Enforce",
    "publisher_missing_key", true);
}
