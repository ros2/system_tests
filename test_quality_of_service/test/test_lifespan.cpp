// Copyright 2019 Open Source Robotics Foundation, Inc.
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
#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "std_msgs/msg/string.hpp"

#include "test_quality_of_service/qos_test_publisher.hpp"
#include "test_quality_of_service/qos_test_subscriber.hpp"
#include "test_quality_of_service/qos_utilities.hpp"

using namespace std::chrono_literals;

TEST_F(QosRclcppTestFixture, test_deadline) {
  const std::chrono::milliseconds lifespan_duration = 1s;
  const int history = 2;
  const std::chrono::milliseconds max_test_length = 11s;
  const int expected_published = 2 * static_cast<int>(max_test_length / lifespan_duration);
  const std::chrono::milliseconds publish_period = 500ms;

  // define qos profile
  rclcpp::QoS qos_profile(history);
  qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
  qos_profile.lifespan(lifespan_duration);

  std::string topic = "test_lifespan";

  publisher = std::make_shared<QosTestPublisher>(
    "publisher", topic, qos_profile, publish_period);
  subscriber = std::make_shared<QosTestSubscriber>(
    "subscriber", topic, qos_profile);

  int timer_fired_count = 0;
  // toggle publishing on and off to force deadline events
  rclcpp::TimerBase::SharedPtr toggle_subscriber_timer = subscriber->create_wall_timer(
    lifespan_duration * 2,
    [this, &timer_fired_count]() -> void {
      // start / stop publishing publish at a rate slower than lifespan
      subscriber->toggle();
      timer_fired_count++;
    });

  executor->add_node(subscriber);
  subscriber->start();

  executor->add_node(publisher);
  publisher->start();

  // the future will never be resolved, so simply time out to force the experiment to stop
  executor->spin_until_future_complete(dummy_future, max_test_length);
  toggle_subscriber_timer->cancel();

  EXPECT_GT(timer_fired_count, 0);
  EXPECT_EQ(publisher->get_count(), expected_published);  // check if we published anything
  EXPECT_GT(subscriber->get_count(), 0);  // check if we received anything

  // check if lifespan simply worked
  EXPECT_LT(subscriber->get_count(), publisher->get_count());
  EXPECT_LT(subscriber->get_count() / 2, publisher->get_count());
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
