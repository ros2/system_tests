// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rmw/event.h"
#include "std_msgs/msg/u_int8_multi_array.hpp"

#include "test_quality_of_service/qos_test_publisher.hpp"
#include "test_quality_of_service/qos_test_subscriber.hpp"
#include "test_quality_of_service/qos_utilities.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::UInt8MultiArray;

namespace
{

constexpr auto kPeriod = 500ms;  // deadline and liveliness lease duration
constexpr auto kPublishInterval = 50ms;
constexpr int kPublishCount = 20;

// Publishes kPublishCount messages, kPublishInterval apart.
void publish_for_a_while(rclcpp::Publisher<UInt8MultiArray> & publisher)
{
  for (int i = 0; i < kPublishCount; ++i) {
    UInt8MultiArray msg;
    msg.data.assign(64, static_cast<uint8_t>(i));
    publisher.publish(msg);
    std::this_thread::sleep_for(kPublishInterval);
  }
}

}  // namespace

/// Test Deadline with a uint8[] type, which an RMW may send on a separate (buffer-aware) path
TEST_F(QosRclcppTestFixture, test_deadline_for_uint8_sequence_type) {
  if (!rmw_event_type_is_supported(RMW_EVENT_OFFERED_DEADLINE_MISSED) ||
    !rmw_event_type_is_supported(RMW_EVENT_REQUESTED_DEADLINE_MISSED))
  {
    GTEST_SKIP();
  }

  auto node = std::make_shared<rclcpp::Node>("test_deadline_for_uint8_sequence_type");
  const std::string topic("test_deadline_for_uint8_sequence_type");
  const auto qos = rclcpp::QoS(10).reliable().deadline(kPeriod);

  std::atomic<int> offered_deadline_missed{0};
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.deadline_callback =
    [&](rclcpp::QOSDeadlineOfferedInfo &) {++offered_deadline_missed;};
  std::atomic<int> requested_deadline_missed{0};
  std::atomic<int> received{0};
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.deadline_callback =
    [&](rclcpp::QOSDeadlineRequestedInfo &) {++requested_deadline_missed;};

  auto sub = node->create_subscription<UInt8MultiArray>(
    topic, qos, [&](const UInt8MultiArray &) {++received;}, sub_options);
  auto pub = node->create_publisher<UInt8MultiArray>(topic, qos, pub_options);

  executor->add_node(node);
  std::thread spinner([this]() {executor->spin();});

  ASSERT_TRUE(wait_for([&]() {return pub->get_subscription_count() == 1u;}, 10s));
  publish_for_a_while(*pub);
  const int missed_while_publishing =
    offered_deadline_missed.load() + requested_deadline_missed.load();

  // Stop publishing: both sides must report missed deadlines.
  const bool reported = wait_for(
    [&]() {return offered_deadline_missed.load() > 0 && requested_deadline_missed.load() > 0;},
    kPeriod * 6);

  executor->cancel();
  spinner.join();

  EXPECT_GT(received.load(), 0);
  EXPECT_EQ(0, missed_while_publishing);
  EXPECT_TRUE(reported) << "offered: " << offered_deadline_missed.load() <<
    ", requested: " << requested_deadline_missed.load();
}

/// Test Liveliness with a uint8[] type, which an RMW may send on a separate (buffer-aware) path
TEST_F(QosRclcppTestFixture, test_liveliness_for_uint8_sequence_type) {
  if (!rmw_event_type_is_supported(RMW_EVENT_LIVELINESS_CHANGED) ||
    !rmw_event_type_is_supported(RMW_EVENT_LIVELINESS_LOST))
  {
    GTEST_SKIP();
  }

  auto node = std::make_shared<rclcpp::Node>("test_liveliness_for_uint8_sequence_type");
  const std::string topic("test_liveliness_for_uint8_sequence_type");
  const auto qos = rclcpp::QoS(10).reliable()
    .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
    .liveliness_lease_duration(kPeriod);

  std::atomic<int> liveliness_lost{0};
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.liveliness_callback =
    [&](rclcpp::QOSLivelinessLostInfo &) {++liveliness_lost;};
  std::atomic<int> alive_count{0};
  std::atomic<int> received{0};
  rclcpp::SubscriptionOptions sub_options;
  sub_options.event_callbacks.liveliness_callback =
    [&](rclcpp::QOSLivelinessChangedInfo & info) {alive_count = info.alive_count;};

  auto sub = node->create_subscription<UInt8MultiArray>(
    topic, qos, [&](const UInt8MultiArray &) {++received;}, sub_options);
  auto pub = node->create_publisher<UInt8MultiArray>(topic, qos, pub_options);

  executor->add_node(node);
  std::thread spinner([this]() {executor->spin();});

  ASSERT_TRUE(wait_for([&]() {return pub->get_subscription_count() == 1u;}, 10s));
  publish_for_a_while(*pub);
  const int alive_while_publishing = alive_count.load();

  // Stop publishing: the publisher loses liveliness.
  const bool reported = wait_for(
    [&]() {return liveliness_lost.load() > 0 && alive_count.load() == 0;}, kPeriod * 6);

  executor->cancel();
  spinner.join();

  EXPECT_GT(received.load(), 0);
  EXPECT_EQ(1, alive_while_publishing) << "the subscription never saw the publisher alive";
  EXPECT_TRUE(reported) << "liveliness lost: " << liveliness_lost.load() <<
    ", alive count: " << alive_count.load();
}
