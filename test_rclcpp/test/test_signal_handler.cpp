// Copyright 2017 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

void sigintHandler(int sig)
{
  (void)sig;
  printf("Custom sigint handler called.\n");
}

int main(int argc, char ** argv)
{
  // This exectuable is used in combination with a script that interrupts it at different times.
  // Its purpose is to test that a user-defined signal handler gets called even when the rclcpp
  // signal handler is/has been in use.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Override default sigint handler
  signal(SIGINT, sigintHandler);
  printf("Registered custom signal handler.\n");

  rclcpp::init(argc, argv);
  printf("Called rclcpp::init.\n");

  printf("Waiting to give an opportunity for interrupt...\n");
  std::this_thread::sleep_for(5s);

  printf("Calling rclcpp::shutdown...\n");
  rclcpp::shutdown();
  printf("Called rclcpp::shutdown.\n");

  printf("Waiting to give an opportunity for interrupt...\n");
  std::this_thread::sleep_for(5s);

  printf("Exiting.\n");
  return 0;
}
