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
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "service_fixtures.hpp"

template<typename T>
int request(
  rclcpp::Node::SharedPtr node,
  const std::string & service_type,
  std::vector<
    std::pair<
      typename T::Request::SharedPtr,
      typename T::Response::SharedPtr
    >
  > services,
  size_t number_of_cycles = 5)
{
  int rc = 0;
  auto start = std::chrono::steady_clock::now();

  auto requester = node->create_client<T>(std::string("test_service_") + service_type);

  rclcpp::WallRate cycle_rate(1);
  auto wait_between_services = std::chrono::milliseconds(100);
  size_t cycle_index = 0;
  size_t service_index = 0;
  // publish the first request up to number_of_cycles times, longer sleep between each cycle
  // publish all requests one by one, shorter sleep between each request
  while (rclcpp::ok() && cycle_index < number_of_cycles && service_index < services.size()) {
    std::cout << "publishing request #" << (service_index + 1) << std::endl;
    auto f = requester->async_send_request(services[service_index].first);

    auto wait_for_response_until = std::chrono::steady_clock::now() + wait_between_services;
    std::future_status status;
    do {
      rclcpp::spin_some(node);
      status = f.wait_for(std::chrono::milliseconds(10));
    } while (
      status != std::future_status::ready && rclcpp::ok() &&
      std::chrono::steady_clock::now() < wait_for_response_until
    );

    if (std::future_status::ready == status) {
      if (*f.get() == *services[service_index].second) {
        std::cout << "received reply #" << (service_index + 1) << " of " <<
          services.size() << std::endl;
        ++service_index;
      } else {
        std::cerr << "received reply for request #" << (service_index + 1) <<
          " does not match the expected reply" << std::endl;
        rc = 2;
        break;
      }
    } else {
      if (service_index == 0) {
        // retry first request up to number_of_cycles times
        ++cycle_index;
        cycle_rate.sleep();
      } else {
        // for not-first requests the reply must arrive without retrying
        rc = 3;
        break;
      }
    }
  }

  if (service_index != services.size()) {
    rc = 1;
  }

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<float> diff = (end - start);
  std::cout << "requested for " << diff.count() << " seconds" << std::endl;

  return rc;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  if (argc != 2) {
    fprintf(stderr, "Wrong number of arguments, pass one service type\n");
    return 1;
  }

  std::string service = argv[1];
  auto node = rclcpp::Node::make_shared(std::string("test_requester_") + service);

  // NOTE(esteve): sleep for two seconds to let the service start up
  std::this_thread::sleep_for(std::chrono::seconds(2));

  int rc;
  if (service == "empty") {
    rc = request<test_communication::srv::Empty>(
      node, service, get_services_empty());
  } else if (service == "primitives") {
    rc = request<test_communication::srv::Primitives>(
      node, service, get_services_primitives());
  } else {
    fprintf(stderr, "Unknown service argument '%s'\n", service.c_str());
    return 1;
  }
  return rc;
}
