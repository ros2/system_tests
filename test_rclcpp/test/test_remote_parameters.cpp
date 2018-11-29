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
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "parameter_fixtures.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

using namespace std::chrono_literals;

TEST(CLASSNAME(parameters, rmw_implementation), test_remote_parameters_async) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  std::string test_server_name = "test_parameters_server";
  // TODO(tfoote) make test_server name parameterizable
  // if (argc >= 2) {
  //   test_server_name = argv[1];
  // }

  auto node = rclcpp::Node::make_shared(std::string("test_remote_parameters_async"));

  auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node,
      test_server_name);
  if (!parameters_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  verify_set_parameters_async(node, parameters_client);

  verify_get_parameters_async(node, parameters_client);
}

TEST(CLASSNAME(parameters, rmw_implementation), test_remote_parameters_sync) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  std::string test_server_name = "test_parameters_server";

  auto node = rclcpp::Node::make_shared(std::string("test_remote_parameters_sync"));

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node,
      test_server_name);
  if (!parameters_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  set_test_parameters(parameters_client);

  verify_test_parameters(parameters_client);
}

TEST(CLASSNAME(parameters, rmw_implementation), test_set_remote_parameters_atomically_sync) {
  if (!rclcpp::ok()) {rclcpp::init(0, nullptr);}
  std::string test_server_name = "test_parameters_server";

  auto node = rclcpp::Node::make_shared(std::string("test_set_remote_parameters_atomically_sync"));

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node,
      test_server_name);
  if (!parameters_client->wait_for_service(20s)) {
    ASSERT_TRUE(false) << "service not available after waiting";
  }

  set_test_parameters_atomically(parameters_client);

  verify_test_parameters(parameters_client);
}
