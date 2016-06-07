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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "service_fixtures.hpp"

template<typename T>
void reply(
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
          std::cout << "received request #" << (index + 1) << " of " <<
            expected_services.size() << std::endl;
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

  node->create_service<T>(std::string("test_service_") + service_type, callback);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    fprintf(stderr, "Wrong number of arguments, pass one service type\n");
    return 1;
  }

  auto start = std::chrono::steady_clock::now();

  std::string service = argv[1];
  auto node = rclcpp::Node::make_shared(std::string("test_replier_") + service);

  auto services_empty = get_services_empty();
  auto services_primitives = get_services_primitives();

  if (service == "Empty") {
    reply<test_communication::srv::Empty>(
      node, service, services_empty);
  } else if (service == "Primitives") {
    reply<test_communication::srv::Primitives>(
      node, service, services_primitives);
  } else {
    fprintf(stderr, "Unknown service argument '%s'\n", service.c_str());
    return 1;
  }
  rclcpp::spin(node);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "replied for " << diff.count() << " seconds" << std::endl;

  return 0;
}
