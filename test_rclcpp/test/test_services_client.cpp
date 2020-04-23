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

#include <chrono>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/srv/add_two_ints.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

using namespace std::chrono_literals;

TEST(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), test_add_noreqid) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = rclcpp::Node::make_shared("test_services_client_no_reqid");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("add_two_ints_noreqid");
  auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request->a = 1;
  request->b = 2;

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(node, result, 5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(3, result.get()->sum);
}

TEST(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), test_add_reqid) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = rclcpp::Node::make_shared("test_services_client_add_reqid");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("add_two_ints_reqid");
  auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request->a = 4;
  request->b = 5;

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(request);

  auto ret = rclcpp::spin_until_future_complete(node, result, 5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);

  EXPECT_EQ(9, result.get()->sum);
}

TEST(CLASSNAME(test_services_client, RMW_IMPLEMENTATION), test_return_request) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  auto node = rclcpp::Node::make_shared("test_services_client_return_request");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>(
    "add_two_ints_reqid_return_request");
  auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request->a = 4;
  request->b = 5;

  if (!client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  auto result = client->async_send_request(
    request,
    [](rclcpp::Client<test_rclcpp::srv::AddTwoInts>::SharedFutureWithRequest future) {
      EXPECT_EQ(4, future.get().first->a);
      EXPECT_EQ(5, future.get().first->b);
      EXPECT_EQ(9, future.get().second->sum);
    });

  auto ret = rclcpp::spin_until_future_complete(node, result, 5s);  // Wait for the result.
  ASSERT_EQ(ret, rclcpp::FutureReturnCode::SUCCESS);
}
