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
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

namespace test_rclcpp
{

// Sleep for timeout ms or until a subscriber has registered for the topic
void wait_for_subscriber(
  std::shared_ptr<rclcpp::Node> node,
  const std::string & topic_name,
  std::chrono::milliseconds timeout = std::chrono::milliseconds(1),
  std::chrono::microseconds sleep_period = std::chrono::seconds(1))
{
  using namespace std::chrono;
  microseconds time_slept(0);
  while (node->count_subscribers(topic_name) == 0 &&
    time_slept < duration_cast<std::chrono::microseconds>(timeout))
  {
    auto start = steady_clock::now();
    rclcpp::event::Event::SharedPtr graph_event = node->get_graph_event();
    node->wait_for_graph_change(graph_event, sleep_period);
    time_slept += duration_cast<std::chrono::microseconds>(steady_clock::now() - start);
  }
  int64_t time_slept_count =
    std::chrono::duration_cast<std::chrono::microseconds>(time_slept).count();
  printf("Waited %" PRId64 " microseconds for the subscriber to connect to topic '%s'\n",
    time_slept_count,
    topic_name.c_str());
}

}  // namespace test_rclcpp

#endif  // TEST_RCLCPP__UTILS_HPP_
