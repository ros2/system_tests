// Copyright 2018 Open Source Robotics Foundation, Inc.
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
#include <cstdio>
#include <iostream>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    fprintf(
      stderr, "Must pass at least one argument with the expected node name\n");
    return 1;
  }
  std::string name_to_find(argv[1]);
  name_to_find = "/" + name_to_find;
  printf("Waiting for node with name: %s\n", name_to_find.c_str());
  std::cout.flush();

  auto node_name = std::string("node_name_list");
  auto node = rclcpp::Node::make_shared(node_name);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  int rc = 1;
  const std::chrono::steady_clock::time_point max_runtime =
    std::chrono::steady_clock::now() + 15s;
  while (rc && rclcpp::ok()) {
    std::vector<std::string> names = node->get_node_names();
    for (const std::string & name : names) {
      printf("- %s\n", name.c_str());

      if (name.empty()) {
        printf("  found an empty named node, which is unexpected\n");
        rc = 2;
        break;
      }

      if (argc >= 2 && name.compare(name_to_find) == 0) {
        printf("  found expected node name\n");
        rc = 0;
      }
    }
    std::cout.flush();
    if (std::chrono::steady_clock::now() >= max_runtime) {
      break;
    }
    exec.spin_some(250ms);
  }

  exec.remove_node(node);
  rclcpp::shutdown();

  if (rc == 1) {
    fprintf(stderr, "not found expected node name\n");
  } else if (rc == 2) {
    fprintf(stderr, "found a node with an empty name\n");
  }
  return rc;
}
