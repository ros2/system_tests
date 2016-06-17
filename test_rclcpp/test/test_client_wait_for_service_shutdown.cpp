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

#include <chrono>
#include <string>  // TODO(wjwwood): remove me when fastrtps exclusion is removed
#include <thread>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rmw/rmw.h"  // TODO(wjwwood): remove me when fastrtps exclusion is removed
#include "test_rclcpp/srv/add_two_ints.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

// rclcpp::shutdown() should wake up wait_for_service, even without spin.
TEST(CLASSNAME(service_client, RMW_IMPLEMENTATION), wait_for_service_shutdown) {
  // TODO(wjwwood): remove this "skip" when fastrtps supports wait_for_service.
  if (std::string(rmw_get_implementation_identifier()) == "rmw_fastrtps_cpp") {
    return;
  }
  rclcpp::init(0, nullptr);
  auto node = rclcpp::node::Node::make_shared("wait_for_service_shutdown");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("wait_for_service_shutdown");

  auto shutdown_thread = std::thread([]() {
    std::this_thread::sleep_for(1_s);
    rclcpp::shutdown();
  });
  auto start = std::chrono::steady_clock::now();
  client->wait_for_service(15_s);
  auto end = std::chrono::steady_clock::now();
  ASSERT_LE(end - start, 10_s);
  shutdown_thread.join();
}
