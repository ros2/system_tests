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

#ifndef TEST_RCLCPP__UTILS_HPP_
#define TEST_RCLCPP__UTILS_HPP_


#include <cinttypes>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#define STRING_(s) #s
#define STRING(s) STRING_(s)

namespace test_rclcpp
{

// Sleep for timeout ms or until a subscriber has registered for the topic
void busy_wait_for_subscriber(
  std::shared_ptr<const rclcpp::Node> node,
  const std::string & topic_name,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1),
  std::chrono::microseconds sleep_period = std::chrono::microseconds(100))
{
#ifdef RMW_IMPLEMENTATION
  if (strcmp(STRING(RMW_IMPLEMENTATION), "rmw_fastrtps_cpp") == 0) {
    printf("FastRTPS detected, sleeping for a fixed interval\n");
    (void)topic_name;
    (void)node;
    (void)sleep_period;
    std::this_thread::sleep_for(timeout);
    return;
  }
#endif
  std::chrono::microseconds time_slept(0);
  while (node->count_subscribers(topic_name) == 0 &&
    time_slept < std::chrono::duration_cast<std::chrono::microseconds>(timeout))
  {
    std::this_thread::sleep_for(sleep_period);
    time_slept += sleep_period;
  }
  int64_t time_slept_count =
    std::chrono::duration_cast<std::chrono::microseconds>(time_slept).count();
  printf("Waited %" PRId64 " microseconds for the subscriber to connect to topic '%s'\n",
    time_slept_count,
    topic_name.c_str());
}


}  // namespace test_rclcpp

#endif  // TEST_RCLCPP__UTILS_HPP_
