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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/service_fixtures.hpp"

template<typename T>
typename rclcpp::service::Service<T>::SharedPtr reply(
  rclcpp::Node::SharedPtr node,
  const std::string & service_type,
  const std::vector<
    std::pair<typename T::Request::SharedPtr, typename T::Response::SharedPtr>> & expected_services
)
{
  // *INDENT-OFF* (prevent uncrustify from making unnecessary indents here)
  auto callback =
    [&expected_services](
      const typename T::Request::SharedPtr received_request,
      typename T::Response::SharedPtr response) -> void
    {
      // find received request in vector of expected services
      bool known_request = false;
      size_t index = 0;
      for (auto expected_service : expected_services) {
        if (*received_request == *expected_service.first) {
          printf("received request #%zu of %zu\n", index + 1, expected_services.size());
          *response = *expected_service.second;
          known_request = true;
          break;
        }
        ++index;
      }
      if (!known_request) {
        fprintf(stderr, "received request does not match any expected request\n");
        rclcpp::shutdown();
        throw std::runtime_error("received request does not match any expected request");
      }
    };
  // *INDENT-ON*

  return node->create_service<T>(std::string("test_service_") + service_type, callback);
}

int main(int argc, char ** argv)
{
  if (argc != 2) {
    fprintf(stderr, "Wrong number of arguments, pass one service type\n");
    return 1;
  }
  rclcpp::init(argc, argv);

  auto start = std::chrono::steady_clock::now();

  std::string service = argv[1];
  auto node = rclcpp::Node::make_shared(std::string("test_replier_") + service);

  auto services_empty = get_services_empty();
  auto services_primitives = get_services_primitives();
  rclcpp::service::ServiceBase::SharedPtr server;

  if (service == "Empty") {
    server = reply<test_msgs::srv::Empty>(
      node, service, services_empty);
  } else if (service == "Primitives") {
    server = reply<test_msgs::srv::Primitives>(
      node, service, services_primitives);
  } else {
    fprintf(stderr, "Unknown service argument '%s'\n", service.c_str());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  printf("replied for %f seconds\n", diff.count());

  rclcpp::shutdown();
  return 0;
}
