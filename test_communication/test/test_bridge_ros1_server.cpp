// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include "ros/ros.h"
#include <diagnostic_msgs/SelfTest.h>

using namespace diagnostic_msgs;

bool add(SelfTest::Request& /* req */, SelfTest::Response& res)
{
  res.id = "ros1";
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros1_bridge_test_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("ros1_bridge_test", add);
  ros::spin();
  return 0;
}
