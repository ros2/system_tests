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

#include "test_quality_of_service/publisher.hpp"
#include "test_quality_of_service/qos_utilities.hpp"
#include "test_quality_of_service/subscriber.hpp"

using namespace std::chrono_literals;

TEST_F(QosRclcppTestFixture, test_deadline) {
  const std::chrono::milliseconds lifespan_duration = 1s;
  const std::tuple<size_t, size_t> message_lifespan = convert_chrono_milliseconds_to_size_t(
    lifespan_duration);
  const int history = 2;
  const std::chrono::milliseconds max_test_length = 11s;
  const int expected_published = max_test_length / lifespan_duration * 2;
  const std::chrono::milliseconds publish_period = 500ms;

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
      publish_period);
  auto subscriber = std::make_shared<Subscriber>("subscriber", topic, subscriber_options);

  // toggle publishing on and off to force deadline events
  rclcpp::TimerBase::SharedPtr toggle_subscriber_timer = subscriber->create_wall_timer(
    lifespan_duration * 2,
    [&subscriber]() -> void {
      // start / stop publishing publish at a rate slower than lifespan
      subscriber->toggle();
    });

  exec->add_node(subscriber);
  subscriber->start();

  exec->add_node(publisher);
  publisher->start();

  // the future will never be resolved, so simply time out to force the experiment to stop
  exec->spin_until_future_complete(dummy_future, max_test_length);

  toggle_subscriber_timer->cancel();
  subscriber->teardown();
  publisher->teardown();

  EXPECT_EQ(publisher->get_count(), expected_published);  // check if we published anything
  EXPECT_GT(subscriber->get_count(), 0);  // check if we received anything

  // check if lifespan simply worked
  EXPECT_LT(subscriber->get_count(), publisher->get_count());
  EXPECT_LT(subscriber->get_count() / 2, publisher->get_count());
}
