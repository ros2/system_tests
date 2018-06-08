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
#include <vector>

#include "rclcpp/rclcpp.hpp"


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  const std::string node_name = "test_parameters_server";
  const std::string namespace_ = "/";
  rclcpp::Context::SharedPtr context =
    rclcpp::contexts::default_context::get_global_default_context();
  const std::vector<std::string> arguments = {};
  const std::vector<rclcpp::Parameter> initial_parameters = {};
  const bool use_global_arguments = true;
  const bool use_intra_process_comms = false;
  const bool start_parameter_services = true;
  const bool allow_undeclared_parameters = true;

  auto node = rclcpp::Node::make_shared(node_name, namespace_, context, arguments,
      initial_parameters, use_global_arguments, use_intra_process_comms, start_parameter_services,
      allow_undeclared_parameters);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
