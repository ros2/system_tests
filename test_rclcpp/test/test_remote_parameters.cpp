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

// NOLINTNEXTLINE(build/namespaces)
using namespace rclcpp::literals;

TEST(CLASSNAME(parameters, rmw_implementation), test_remote_parameters) {
  std::string test_server_name = "test_parameters_server";
  // TODO(tfoote) make test_server name parameterizable
  // if (argc >= 2) {
  //   test_server_name = argv[1];
  // }

  auto node = rclcpp::Node::make_shared(std::string("test_remote_parameters"));

  // TODO(wjwwood): remove this block when there is a wait_for_parameter_server option.
  {
    // This is requried specifically for FastRTPS, see:
    //   https://github.com/eProsima/ROS-RMW-Fast-RTPS-cpp/pull/51#issuecomment-242872096
    std::this_thread::sleep_for(1_s);
  }

  auto parameters_client = std::make_shared<rclcpp::parameter_client::AsyncParametersClient>(node,
      test_server_name);

  verify_set_parameters_async(node, parameters_client);

  verify_get_parameters_async(node, parameters_client);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
