// Copyright 2023 Open Source Robotics Foundation, Inc.
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

// Regression test for https://github.com/ros2/rmw_fastrtps/issues/715

#include <chrono>
#include <limits>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "rcpputils/scope_exit.hpp"

#include "test_communication/msg/outer.hpp"

template<class T>
static typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp)
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::fabs(x - y) <= std::numeric_limits<T>::epsilon() * std::fabs(x + y) * ulp
         // unless the result is subnormal
         || std::fabs(x - y) < std::numeric_limits<T>::min();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCPPUTILS_SCOPE_EXIT(
  {
    rclcpp::shutdown();
  });

  auto node = rclcpp::Node::make_shared("test_nested_types");

  std::thread spin_thread([node]() {
      rclcpp::spin(node);
    });

  bool valid = false;
  bool received = false;
  auto callback = [&valid, &received](const test_communication::msg::Outer & msg) {
      if (almost_equal(msg.inner.float_field, 1.23456789, 2) &&
        msg.inner.bool_field && msg.bool_field_1 && msg.bool_field_2 && msg.bool_field_3)
      {
        valid = true;
      }

      received = true;
    };
  auto subscription =
    node->create_subscription<test_communication::msg::Outer>("nested_type_topic", 10, callback);

  auto publisher = node->create_publisher<test_communication::msg::Outer>("nested_type_topic", 10);

  auto start = std::chrono::steady_clock::now();
  rclcpp::WallRate cycle_rate(20);
  while (!received && ((std::chrono::steady_clock::now() - start) < std::chrono::seconds(10))) {
    auto msg = test_communication::msg::Outer();
    msg.inner.float_field = 1.23456789;
    msg.inner.bool_field = true;
    msg.bool_field_1 = true;
    msg.bool_field_2 = true;
    msg.bool_field_3 = true;
    publisher->publish(msg);

    cycle_rate.sleep();
  }

  rclcpp::shutdown();

  spin_thread.join();

  if (valid) {
    return 0;
  } else {
    return 1;
  }
}
