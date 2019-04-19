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
#include "test_quality_of_service/qos_utilities.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <memory>
#include <stack>
#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;

/// Liveliness setup
class LivelinessSetup : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  }
  void TearDown() override
  {
    executor->cancel();
    executor.reset();
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};

/// Test Automatic Liveliness with a single publishing node and single subscriber node
TEST_F(LivelinessSetup, test_automatic_liveliness_changed) {
  const std::chrono::milliseconds  max_test_length = 8s;
  const std::chrono::milliseconds  kill_publisher_after = 2s;
  int number_of_published_messages = 0;

  // used for lambda capture
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> exec = executor;
  int total_number_of_liveliness_events = 0;

  rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
  // define liveliness policy
  qos_profile.liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;

  std::tuple<size_t, size_t> liveliness_lease_tuple = chrono_milliseconds_to_size_t(1000ms);

  // setup liveliness lease
  qos_profile.liveliness_lease_duration.sec = std::get<0>(liveliness_lease_tuple);
  qos_profile.liveliness_lease_duration.nsec = std::get<1>(liveliness_lease_tuple);

  // subscription options
  rclcpp::SubscriptionOptions<> subscriber_options;
  subscriber_options.qos_profile = qos_profile;

  // subscriber Liveliness callback event
  subscriber_options.event_callbacks.liveliness_callback =
    [&total_number_of_liveliness_events](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "QOSLivelinessChangedInfo callback");
      total_number_of_liveliness_events++;

      // strict checking for expected events
      if (total_number_of_liveliness_events == 1) {
        // publisher came alive
        ASSERT_EQ(1, event.alive_count);
        ASSERT_EQ(0, event.not_alive_count);
        ASSERT_EQ(1, event.alive_count_change);
        ASSERT_EQ(0, event.not_alive_count_change);
      } else if (total_number_of_liveliness_events == 2) {
        // publisher died
        ASSERT_EQ(0, event.alive_count);
        ASSERT_EQ(0, event.not_alive_count);
        ASSERT_EQ(-1, event.alive_count_change);
        ASSERT_EQ(0, event.not_alive_count_change);
      }
    };

  // publisher options
  rclcpp::PublisherOptions<> publisher_options;
  publisher_options.qos_profile = qos_profile;

//  publisher_options.event_callbacks.liveliness_callback =
//    [](rclcpp::QOSLivelinessLostInfo & event) -> void
//    {
//      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "=====QOSLivelinessLostInfo callback fired");
//    };

  std::string topic("test_automatic_liveliness_changed");

  auto publisher = std::make_shared<Publisher>("publisher", topic, publisher_options, 200ms);

  auto subscriber = std::make_shared<Subscriber>("subscriber", topic, subscriber_options);

  // kill the test after a predetermined amount of time
  rclcpp::TimerBase::SharedPtr kill_publisher_timer = nullptr;
  kill_publisher_timer = subscriber->create_wall_timer(
    2s,
    [&exec, &publisher, &number_of_published_messages, &kill_publisher_timer]() -> void {
      exec->remove_node(publisher);
      number_of_published_messages = publisher->sent_message_count_;
      publisher.reset();  // force liveliness lost for automatic
      kill_publisher_timer->cancel();
    });

  // stop the experiment after a predetermined amount of time
  rclcpp::TimerBase::SharedPtr kill_experiment_timer = nullptr;
  kill_experiment_timer = subscriber->create_wall_timer(
    max_test_length,
    [&exec, &kill_experiment_timer]() -> void {
      // don't cancel timer one from here....
      exec->cancel();
      kill_experiment_timer->cancel();
    });

  exec->add_node(subscriber);
  subscriber->start_listening();

  exec->add_node(publisher);
  publisher->start_publishing();

  exec->spin();  // blocking

  EXPECT_EQ(2, total_number_of_liveliness_events);  // check expected number of liveliness events
  EXPECT_GT(number_of_published_messages, 0);  // check if we published anything
  EXPECT_GT(subscriber->received_message_count_, 0);  // check if we received anything
}

// todo multiple publisher test?

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  return ret;
}
