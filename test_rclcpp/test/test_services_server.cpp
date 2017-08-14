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

#include <rclcpp/rclcpp.hpp>

#include <test_rclcpp/srv/add_two_ints.hpp>

#include <memory>

void handle_add_two_ints_noreqid(
  const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
{
  response->sum = request->a + request->b;
}

void handle_add_two_ints_reqid(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
  std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
{
  (void)request_header;
  response->sum = request->a + request->b;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("add_two_ints_server");

  auto service_noreqid = node->create_service<test_rclcpp::srv::AddTwoInts>(
    "add_two_ints_noreqid", handle_add_two_ints_noreqid);

  auto service_reqid = node->create_service<test_rclcpp::srv::AddTwoInts>(
    "add_two_ints_reqid", handle_add_two_ints_reqid);

  auto service_return_req = node->create_service<test_rclcpp::srv::AddTwoInts>(
    "add_two_ints_reqid_return_request", handle_add_two_ints_reqid);

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
