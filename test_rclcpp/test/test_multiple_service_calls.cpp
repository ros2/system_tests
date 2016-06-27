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
#include <thread>  // TODO(wjwwood): remove me when Connext and FastRTPS exclusions are removed
#include <utility>
#include <vector>

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

void handle_add_two_ints(
  const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
}

TEST(CLASSNAME(test_two_service_calls, RMW_IMPLEMENTATION), two_service_calls) {
  auto node = rclcpp::Node::make_shared("test_two_service_calls");

  node->create_service<test_rclcpp::srv::AddTwoInts>(
    "test_two_service_calls", handle_add_two_ints);

  auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("test_two_service_calls");
  {  // TODO(wjwwood): remove this block when Connext and FastRTPS support wait_for_service.
    try {
      if (!client->wait_for_service(20_s)) {
        ASSERT_TRUE(false) << "service not available after waiting";
      }
    } catch (rclcpp::exceptions::RCLError) {
      std::this_thread::sleep_for(1_s);
    }
  }

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
  // The first result should still be 1.
  EXPECT_EQ(1, result1.get()->sum);
}

TEST(CLASSNAME(test_multiple_service_calls, RMW_IMPLEMENTATION), multiple_clients) {
  const uint32_t n = 5;

  auto node = rclcpp::Node::make_shared("test_multiple_clients");
  rclcpp::executors::SingleThreadedExecutor executor;

  node->create_service<test_rclcpp::srv::AddTwoInts>(
    "test_multiple_clients", handle_add_two_ints);

  using ClientRequestPair = std::pair<
      rclcpp::client::Client<test_rclcpp::srv::AddTwoInts>::SharedPtr,
      test_rclcpp::srv::AddTwoInts::Request::SharedPtr>;
  using SharedFuture = rclcpp::client::Client<test_rclcpp::srv::AddTwoInts>::SharedFuture;

  std::vector<ClientRequestPair> client_request_pairs;
  for (uint32_t i = 0; i < n; ++i) {
    auto client = node->create_client<test_rclcpp::srv::AddTwoInts>("test_multiple_clients");
    auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
    request->a = i;
    request->b = i + 1;
    client_request_pairs.push_back(ClientRequestPair(client, request));
  }

  std::vector<SharedFuture> results;

  printf("Sending %u requests...\n", n);
  fflush(stdout);
  // Send all the requests
  for (auto & pair : client_request_pairs) {
    {  // TODO(wjwwood): remove this block when Connext and FastRTPS support wait_for_service.
      try {
        if (!pair.first->wait_for_service(20_s)) {
          ASSERT_TRUE(false) << "service not available after waiting";
        }
      } catch (rclcpp::exceptions::RCLError) {
        std::this_thread::sleep_for(1_s);
      }
    }
    results.push_back(pair.first->async_send_request(pair.second));
  }

  auto timer_callback = [&executor, &results]() {
      bool all_ready = true;
      for (auto & result : results) {
        all_ready &= result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
      }
      if (all_ready) {
        executor.cancel();
      }
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(3), timer_callback);

  executor.add_node(node);

  executor.spin();

  // Check the status of all futures
  for (uint32_t i = 0; i < results.size(); ++i) {
    ASSERT_EQ(std::future_status::ready, results[i].wait_for(std::chrono::seconds(0)));
    EXPECT_EQ(results[i].get()->sum, 2 * i + 1);
    printf("Got response #%u with value %zd\n", i, results[i].get()->sum);
    fflush(stdout);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
