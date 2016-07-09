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

#include <iostream>
#include <memory>
#include <thread>  // TODO(wjwwood): remove me when Connext and FastRTPS exclusions are removed

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

// This test is concerned with the consistency of the two clients' behavior, not necessarily whether
// or not they are successful.
TEST(CLASSNAME(service_client, RMW_IMPLEMENTATION), client_scope_consistency_regression_test) {
  auto node = rclcpp::node::Node::make_shared("client_scope_consistency_regression_test");

  // Replicate the settings that caused https://github.com/ros2/system_tests/issues/153
  rmw_qos_profile_t rmw_qos_profile =
  {
    RMW_QOS_POLICY_KEEP_LAST_HISTORY,
    10,
    RMW_QOS_POLICY_RELIABLE,
    RMW_QOS_POLICY_VOLATILE_DURABILITY
  };
  rclcpp::executor::FutureReturnCode ret1;

  // Extra scope so the first client will be deleted afterwards
  {
    printf("creating first client\n");
    std::cout.flush();
    auto client1 = node->create_client<test_rclcpp::srv::AddTwoInts>(
      "client_scope", rmw_qos_profile);
    {  // TODO(wjwwood): remove this block when Connext and FastRTPS support wait_for_service.
      try {
        if (!client1->wait_for_service(20_s)) {
          ASSERT_TRUE(false) << "service not available after waiting";
        }
      } catch (rclcpp::exceptions::RCLError) {
        std::this_thread::sleep_for(1_s);
      }
    }
    auto request1 = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
    request1->a = 1;
    request1->b = 2;

    printf("sending first request\n");
    std::cout.flush();
    auto result1 = client1->async_send_request(request1);

    ret1 = rclcpp::spin_until_future_complete(node, result1, 5_s);
    if (ret1 == rclcpp::executor::FutureReturnCode::SUCCESS) {
      printf("received first result\n");
      std::cout.flush();
      if (3 == result1.get()->sum) {
        printf("receivied correct result\n");
        std::cout.flush();
      } else {
        printf("received incorrect result: %li\n", result1.get()->sum);
        std::cout.flush();
      }
    } else {
      printf("first result not received: %s\n", rclcpp::executor::to_string(ret1).c_str());
      std::cout.flush();
    }

    printf("destroying first client\n");
    std::cout.flush();
  }
  {
    printf("creating second client\n");
    std::cout.flush();

    auto client2 = node->create_client<test_rclcpp::srv::AddTwoInts>(
      "client_scope", rmw_qos_profile);
    {  // TODO(wjwwood): remove this block when Connext and FastRTPS support wait_for_service.
      try {
        if (!client2->wait_for_service(20_s)) {
          ASSERT_TRUE(false) << "service not available after waiting";
        }
      } catch (rclcpp::exceptions::RCLError) {
        std::this_thread::sleep_for(1_s);
      }
    }
    auto request2 = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
    request2->a = 2;
    request2->b = 3;

    printf("sending second request\n");
    std::cout.flush();
    auto result2 = client2->async_send_request(request2);

    auto ret2 = rclcpp::spin_until_future_complete(node, result2, 5_s);
    if (ret2 == rclcpp::executor::FutureReturnCode::SUCCESS) {
      printf("received second result\n");
      std::cout.flush();
      if (5 == result2.get()->sum) {
        printf("receivied correct result\n");
        std::cout.flush();
      } else {
        printf("received incorrect result: %li\n", result2.get()->sum);
        std::cout.flush();
      }
    } else {
      printf("second result not received: %s\n", rclcpp::executor::to_string(ret2).c_str());
      std::cout.flush();
    }

    ASSERT_EQ(ret2, ret1) << "Both clients should have the same behavior.";

    printf("destroying second client\n");
    std::cout.flush();
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
