// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <iostream>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <test_rclcpp/srv/add_two_ints.hpp>

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

void handle_add_two_ints(
  const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
}

TEST(CLASSNAME(test_multiple_service_calls, RMW_IMPLEMENTATION), multiple_service_calls) {
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_multiple_service_calls");

  node->create_service<test_rclcpp::srv::AddTwoInts>(
    "test_multiple_service_calls", handle_add_two_ints);

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("test_multiple_service_calls");

  auto request1 = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request1->a = 1;
  request1->b = 0;

  auto request2 = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request2->a = 2;
  request2->b = 0;

  printf("Sending two requests...\n");
  fflush(stdout);

  auto result1 = client->async_send_request(request1);
  auto result2 = client->async_send_request(request2);

  printf("Waiting for first reply...\n");
  fflush(stdout);
  rclcpp::spin_until_future_complete(node, result1);
  printf("Received first reply\n");
  EXPECT_EQ(1, result1.get()->sum);

  printf("Waiting for second reply...\n");
  fflush(stdout);
  rclcpp::spin_until_future_complete(node, result2);
  printf("Received second reply\n");
  EXPECT_EQ(2, result2.get()->sum);
}
