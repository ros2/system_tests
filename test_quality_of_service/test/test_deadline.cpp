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

#include "test_quality_of_service/qos_utilities.hpp"


using namespace std::chrono_literals;

/// Test Deadline with a single publishing node and single subscriber node
TEST_F(TestSetup, test_deadline) {
  int expected_number_of_pub_events = 5;
  const std::chrono::milliseconds deadline_duration = 1s;
  std::tuple<size_t, size_t> deadline_duration_tuple = chrono_milliseconds_to_size_t(
    deadline_duration);
  const std::chrono::milliseconds max_test_length = 10s;
  const std::chrono::milliseconds publish_rate = deadline_duration / 5;

  // used for lambda capture
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec = executor;
  int total_number_of_publisher_deadline_events = 0;
  int total_number_of_subscriber_deadline_events = 0;
  int last_pub_count = 0;
  int last_sub_count = 0;

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  qos_profile.deadline.sec = std::get<0>(deadline_duration_tuple);
  qos_profile.deadline.nsec = std::get<1>(deadline_duration_tuple);

  // setup subscription options and callback
  rclcpp::SubscriptionOptions<> subscriber_options;
  subscriber_options.qos_profile = qos_profile;
  subscriber_options.event_callbacks.deadline_callback =
    [&last_pub_count,
    &total_number_of_subscriber_deadline_events](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "QOSDeadlineRequestedInfo callback");
      total_number_of_subscriber_deadline_events++;
      // assert the correct value on a change
      ASSERT_EQ(1, event.total_count_change);
      last_pub_count = event.total_count;
    };

  // setup publishing options and callback
  rclcpp::PublisherOptions<> publisher_options;
  publisher_options.qos_profile = qos_profile;
  publisher_options.event_callbacks.deadline_callback =
    [&last_sub_count,
    &total_number_of_publisher_deadline_events](rclcpp::QOSDeadlineOfferedInfo & event) -> void
    {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "QOSDeadlineOfferedInfo callback");
      total_number_of_publisher_deadline_events++;
      // assert the correct value on a change
      ASSERT_EQ(1, event.total_count_change);
      last_sub_count = event.total_count;
    };

  std::string topic("test_deadline");

  auto publisher = std::make_shared<Publisher>("publisher", topic, publisher_options, publish_rate);
  auto subscriber = std::make_shared<Subscriber>("subscriber", topic, subscriber_options);

  // toggle publishing on and off to force deadline events
  rclcpp::TimerBase::SharedPtr kill_publisher_timer = nullptr;
  kill_publisher_timer = subscriber->create_wall_timer(
    max_test_length / expected_number_of_pub_events,
    [&publisher]() -> void {
      // start / stop publishing to trigger deadline
      publisher->toggle_publishing();
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
  exec->add_node(publisher);

  subscriber->start_listening();
  publisher->start_publishing();

  exec->spin();  // blocking

  EXPECT_GT(publisher->sent_message_count_, 0);  // check if we published anything
  EXPECT_GT(subscriber->received_message_count_, 0);  // check if we received anything

  // check to see if callbacks fired as expected
  EXPECT_EQ(expected_number_of_pub_events, total_number_of_subscriber_deadline_events);
  EXPECT_EQ((expected_number_of_pub_events - 1), total_number_of_publisher_deadline_events);

  // check values reported by the callback
  EXPECT_EQ(expected_number_of_pub_events, last_pub_count);
  EXPECT_EQ((expected_number_of_pub_events - 1), last_sub_count);
}
