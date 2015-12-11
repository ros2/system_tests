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
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/srv/add_two_ints.hpp"

TEST(test_services_client, test_add_noreqid) {
  auto node = rclcpp::Node::make_shared("test_services_client");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("add_two_ints_noreqid");
  auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request->a = 1;
  request->b = 2;

  // wait a moment for everything to initialize
  // TODO(richiprosima): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);


  auto result = client->async_send_request(request);

  rclcpp::spin_until_future_complete(node, result);  // Wait for the result.

  EXPECT_EQ(3, result.get()->sum);
}

TEST(test_services_client, test_add_reqid) {
  auto node = rclcpp::Node::make_shared("test_services_client");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("add_two_ints_reqid");
  auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request->a = 4;
  request->b = 5;

  // wait a moment for everything to initialize
  // TODO(richiprosima): fix nondeterministic startup behavior
  rclcpp::utilities::sleep_for(1_ms);

  auto result = client->async_send_request(request);

  rclcpp::spin_until_future_complete(node, result);  // Wait for the result.

  EXPECT_EQ(9, result.get()->sum);
}

TEST(test_services_client, test_return_request) {
  auto node = rclcpp::Node::make_shared("test_services_client");

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("add_two_ints_reqid");
  auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
  request->a = 4;
  request->b = 5;

  auto result = client->async_send_request(
    request,
    [](rclcpp::client::Client<test_rclcpp::srv::AddTwoInts>::SharedFutureWithRequest future) {
    EXPECT_EQ(4, future.get().first->a);
    EXPECT_EQ(5, future.get().first->b);
    EXPECT_EQ(9, future.get().second->sum);
  });

  rclcpp::spin_until_future_complete(node, result);  // Wait for the result.
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
