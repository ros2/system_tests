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

#include "gtest/gtest.h"

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "std_msgs/msg/string.hpp"

#include "test_quality_of_service/qos_utilities.hpp"

using namespace std::chrono_literals;

TEST_F(TestSetup, test_deadline) {

  std::chrono::milliseconds lifespan_duration = 1000ms;
  std::tuple<size_t, size_t> message_lifespan = chrono_milliseconds_to_size_t(lifespan_duration);
  const int history = 2;
  const std::chrono::milliseconds max_test_length = 10500ms;
  const int expected_published = max_test_length / lifespan_duration * 2;

  // used for lambda capture
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec = executor;

  // define qos profile
  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;

  qos_profile.depth = history;
  qos_profile.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;

  qos_profile.lifespan.sec = std::get<0>(message_lifespan);
  qos_profile.lifespan.nsec = std::get<1>(message_lifespan);

  // subscription options
  rclcpp::SubscriptionOptions<> subscriber_options;
  subscriber_options.qos_profile = qos_profile;

  // publisher options
  rclcpp::PublisherOptions<> publisher_options;
  publisher_options.qos_profile = qos_profile;

  std::string topic = "test_lifespan";

  auto publisher = std::make_shared<Publisher>("publisher", topic, publisher_options,
      lifespan_duration / 2);
  auto subscriber = std::make_shared<Subscriber>("subscriber", topic, subscriber_options);

  // toggle publishing on and off to force deadline events
  rclcpp::TimerBase::SharedPtr kill_publisher_timer = nullptr;
  kill_publisher_timer = subscriber->create_wall_timer(
    lifespan_duration * 2,
    [&subscriber]() -> void {
      // start / stop publishing publish at a rate slower than lifespan
      subscriber->toggle_listening();
    });

  // stop the experiment after a predetermined amount of time
  rclcpp::TimerBase::SharedPtr kill_experiment_timer = nullptr;
  kill_experiment_timer = subscriber->create_wall_timer(
    max_test_length,
    [&exec, &kill_experiment_timer]() -> void {
      exec->cancel();
      kill_experiment_timer->cancel();
    });

  exec->add_node(subscriber);
  subscriber->start_listening();

  exec->add_node(publisher);
  publisher->start_publishing();

  exec->spin();  // blocking

  EXPECT_EQ(publisher->sent_message_count_, expected_published);  // check if we published anything
  EXPECT_GT(subscriber->received_message_count_, 0);  // check if we received anything

  // check if lifespan simply worked
  EXPECT_LT(subscriber->received_message_count_, publisher->sent_message_count_);
  EXPECT_LT(subscriber->received_message_count_ / 2, publisher->sent_message_count_);
}
