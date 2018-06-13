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

#ifndef PARAMETER_FIXTURES_HPP_
#define PARAMETER_FIXTURES_HPP_

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include "gtest/gtest.h"

#include "rcl_interfaces/srv/list_parameters.hpp"
#include "rclcpp/rclcpp.hpp"

const double test_epsilon = 1e-6;

void set_test_parameters(
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client)
{
  printf("Setting parameters\n");
  // Set several differnet types of parameters.
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("barstr", std::string("hello_str")),
    rclcpp::Parameter("baz", 1.45),
    rclcpp::Parameter("foo.first", 8),
    rclcpp::Parameter("foo.second", 42),
    rclcpp::Parameter("foobar", true),
  });
  printf("Got set_parameters result\n");


  // Check to see if they were set.
  for (auto & result : set_parameters_results) {
    ASSERT_TRUE(result.successful);
  }
}

void set_test_parameters_atomically(
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client)
{
  printf("Setting parameters atomically\n");
  // Set several differnet types of parameters.
  auto set_parameters_result = parameters_client->set_parameters_atomically({
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("barstr", std::string("hello_str")),
    rclcpp::Parameter("baz", 1.45),
    rclcpp::Parameter("foo.first", 8),
    rclcpp::Parameter("foo.second", 42),
    rclcpp::Parameter("foobar", true),
  });
  printf("Got set_parameters_atomically result\n");

  // Check to see if they were set.
  ASSERT_TRUE(set_parameters_result.successful);
}

void verify_set_parameters_async(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client)
{
  printf("Setting parameters\n");
  // Set several differnet types of parameters.
  auto set_parameters_results = parameters_client->set_parameters({
    rclcpp::Parameter("foo", 2),
    rclcpp::Parameter("bar", "hello"),
    rclcpp::Parameter("barstr", std::string("hello_str")),
    rclcpp::Parameter("baz", 1.45),
    rclcpp::Parameter("foo.first", 8),
    rclcpp::Parameter("foo.second", 42),
    rclcpp::Parameter("foobar", true),
  });
  rclcpp::spin_until_future_complete(node, set_parameters_results);  // Wait for the results.
  printf("Got set_parameters result\n");

  // Check to see if they were set.
  for (auto & result : set_parameters_results.get()) {
    ASSERT_TRUE(result.successful);
  }
}

void verify_test_parameters(
  std::shared_ptr<rclcpp::SyncParametersClient> parameters_client)
{
  printf("Listing parameters with recursive depth\n");
  // Test recursive depth (=0)
  auto parameters_and_prefixes = parameters_client->list_parameters({"foo", "bar"},
      rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  for (auto & name : parameters_and_prefixes.names) {
    EXPECT_TRUE(name == "foo" || name == "bar" || name == "foo.first" || name == "foo.second");
  }
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }

  printf("Listing parameters with depth of 1\n");
  // Test different depth
  auto parameters_and_prefixes4 = parameters_client->list_parameters({"foo"}, 1);
  for (auto & name : parameters_and_prefixes4.names) {
    EXPECT_EQ(name, "foo");
  }
  for (auto & prefix : parameters_and_prefixes4.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }

  printf("Listing parameters with depth of 2\n");
  // Test different depth
  auto parameters_and_prefixes5 = parameters_client->list_parameters({"foo"}, 2);
  for (auto & name : parameters_and_prefixes5.names) {
    EXPECT_TRUE(name == "foo" || name == "foo.first" || name == "foo.second");
  }
  for (auto & prefix : parameters_and_prefixes5.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }

  printf("Getting parameters\n");
  // Get a few of the parameters just set.
  for (auto & parameter : parameters_client->get_parameters({"foo", "bar", "baz"})) {
    // std::cout << "Parameter is:" << std::endl << parameter.to_yaml() << std::endl;
    if (parameter.get_name() == "foo") {
      EXPECT_STREQ(
        "{\"name\": \"foo\", \"type\": \"integer\", \"value\": \"2\"}",
        std::to_string(parameter).c_str());
      EXPECT_STREQ("integer", parameter.get_type_name().c_str());
    } else if (parameter.get_name() == "bar") {
      EXPECT_STREQ(
        "{\"name\": \"bar\", \"type\": \"string\", \"value\": \"hello\"}",
        std::to_string(parameter).c_str());
      EXPECT_STREQ("string", parameter.get_type_name().c_str());
    } else if (parameter.get_name() == "baz") {
      EXPECT_STREQ("double", parameter.get_type_name().c_str());
      EXPECT_NEAR(1.45, parameter.as_double(), test_epsilon);
    } else {
      ASSERT_FALSE("you should never hit this");
    }
  }

  printf("Getting nonexistent parameters\n");
  // Get a few non existant parameters
  {
    std::vector<rclcpp::Parameter> retrieved_params =
      parameters_client->get_parameters({"not_foo", "not_baz"});
    ASSERT_EQ(2u, retrieved_params.size());
    EXPECT_STREQ("not_foo", retrieved_params[0].get_name().c_str());
    EXPECT_STREQ("not_baz", retrieved_params[1].get_name().c_str());
    EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, retrieved_params[0].get_type());
    EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, retrieved_params[1].get_type());
  }

  printf("Listing parameters with recursive depth\n");
  // List all of the parameters, using an empty prefix list and depth=0
  parameters_and_prefixes = parameters_client->list_parameters({},
      rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  std::vector<std::string> all_names = {
    "foo", "bar", "barstr", "baz", "foo.first", "foo.second", "foobar"
  };
  EXPECT_EQ(parameters_and_prefixes.names.size(), all_names.size());
  for (auto & name : all_names) {
    EXPECT_NE(std::find(
        parameters_and_prefixes.names.cbegin(),
        parameters_and_prefixes.names.cend(),
        name),
      parameters_and_prefixes.names.cend());
  }
  printf("Listing parameters with depth 100\n");
  // List all of the parameters, using an empty prefix list and large depth
  parameters_and_prefixes = parameters_client->list_parameters({}, 100);
  EXPECT_EQ(parameters_and_prefixes.names.size(), all_names.size());
  for (auto & name : all_names) {
    EXPECT_NE(std::find(
        parameters_and_prefixes.names.cbegin(),
        parameters_and_prefixes.names.cend(),
        name),
      parameters_and_prefixes.names.cend());
  }
  printf("Listing parameters with depth 1\n");
  // List most of the parameters, using an empty prefix list and depth=1
  parameters_and_prefixes = parameters_client->list_parameters({}, 1);
  std::vector<std::string> depth_one_names = {
    "foo", "bar", "barstr", "baz", "foobar"
  };
  EXPECT_EQ(parameters_and_prefixes.names.size(), depth_one_names.size());
  for (auto & name : depth_one_names) {
    EXPECT_NE(std::find(
        parameters_and_prefixes.names.cbegin(),
        parameters_and_prefixes.names.cend(),
        name),
      parameters_and_prefixes.names.cend());
  }
}

void verify_get_parameters_async(
  std::shared_ptr<rclcpp::Node> node,
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client)
{
  printf("Listing parameters with recursive depth\n");
  // Test recursive depth (=0)
  auto result = parameters_client->list_parameters({"foo", "bar"},
      rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  rclcpp::spin_until_future_complete(node, result);
  auto parameters_and_prefixes = result.get();
  for (auto & name : parameters_and_prefixes.names) {
    EXPECT_TRUE(name == "foo" || name == "bar" || name == "foo.first" || name == "foo.second");
  }
  for (auto & prefix : parameters_and_prefixes.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }

  printf("Listing parameters with depth 1\n");
  // Test different depth
  auto result4 = parameters_client->list_parameters({"foo"}, 1);
  rclcpp::spin_until_future_complete(node, result4);
  auto parameters_and_prefixes4 = result4.get();
  for (auto & name : parameters_and_prefixes4.names) {
    EXPECT_EQ(name, "foo");
  }
  for (auto & prefix : parameters_and_prefixes4.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }

  // Test different depth
  printf("Listing parameters with depth 2\n");
  auto result4a = parameters_client->list_parameters({"foo"}, 2);
  rclcpp::spin_until_future_complete(node, result4a);
  auto parameters_and_prefixes4a = result4a.get();
  for (auto & name : parameters_and_prefixes4a.names) {
    EXPECT_TRUE(name == "foo" || name == "foo.first" || name == "foo.second");
  }
  for (auto & prefix : parameters_and_prefixes4a.prefixes) {
    EXPECT_STREQ("foo", prefix.c_str());
  }


  printf("Getting parameters\n");
  // Get a few of the parameters just set.
  auto result2 = parameters_client->get_parameters({"foo", "bar", "baz"});
  rclcpp::spin_until_future_complete(node, result2);
  for (auto & parameter : result2.get()) {
    if (parameter.get_name() == "foo") {
      EXPECT_STREQ("foo", parameter.get_name().c_str());
      EXPECT_STREQ(
        "{\"name\": \"foo\", \"type\": \"integer\", \"value\": \"2\"}",
        std::to_string(parameter).c_str());
      EXPECT_STREQ("integer", parameter.get_type_name().c_str());
    } else if (parameter.get_name() == "bar") {
      EXPECT_STREQ(
        "{\"name\": \"bar\", \"type\": \"string\", \"value\": \"hello\"}",
        std::to_string(parameter).c_str());
      EXPECT_STREQ("string", parameter.get_type_name().c_str());
    } else if (parameter.get_name() == "baz") {
      EXPECT_STREQ("double", parameter.get_type_name().c_str());
      EXPECT_NEAR(1.45, parameter.as_double(), test_epsilon);
    } else {
      ASSERT_FALSE("you should never hit this");
    }
  }

  printf("Getting nonexistent parameters\n");
  // Get a few non existant parameters
  {
    auto result3 = parameters_client->get_parameters({"not_foo", "not_baz"});
    rclcpp::spin_until_future_complete(node, result3);
    std::vector<rclcpp::Parameter> retrieved_params = result3.get();
    ASSERT_EQ(2u, retrieved_params.size());
    EXPECT_STREQ("not_foo", retrieved_params[0].get_name().c_str());
    EXPECT_STREQ("not_baz", retrieved_params[1].get_name().c_str());
    EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, retrieved_params[0].get_type());
    EXPECT_EQ(rclcpp::ParameterType::PARAMETER_NOT_SET, retrieved_params[1].get_type());
  }

  printf("Listing parameters with recursive depth\n");
  // List all of the parameters, using an empty prefix list
  auto result5 = parameters_client->list_parameters({},
      rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  rclcpp::spin_until_future_complete(node, result5);
  parameters_and_prefixes = result5.get();
  std::vector<std::string> all_names = {
    "foo", "bar", "barstr", "baz", "foo.first", "foo.second", "foobar"
  };
  EXPECT_EQ(parameters_and_prefixes.names.size(), all_names.size());
  for (auto & name : all_names) {
    EXPECT_NE(std::find(
        parameters_and_prefixes.names.cbegin(),
        parameters_and_prefixes.names.cend(),
        name),
      parameters_and_prefixes.names.cend());
  }
}

#endif  // PARAMETER_FIXTURES_HPP_
