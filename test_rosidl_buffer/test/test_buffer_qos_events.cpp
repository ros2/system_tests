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

// Deadline and liveliness events for a subscription receiving through the
// `test` backend, i.e. on the accelerated buffer channel.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcpputils/scope_exit.hpp"
#include "rmw/event.h"
#include "rosidl_buffer/buffer.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

using namespace std::chrono_literals;
using test_rosidl_buffer::msg::ByteArray;

namespace
{

// Waits until predicate() is true or the timeout expires.
template<typename Predicate>
bool wait_for(Predicate predicate, std::chrono::milliseconds timeout)
{
  const auto end = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() > end) {
      return false;
    }
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

}  // namespace

class TestBufferQosEvents : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestBufferQosEvents, deadline_and_liveliness_through_accelerated_backend) {
  for (auto type : {RMW_EVENT_OFFERED_DEADLINE_MISSED, RMW_EVENT_REQUESTED_DEADLINE_MISSED,
      RMW_EVENT_LIVELINESS_LOST, RMW_EVENT_LIVELINESS_CHANGED})
  {
    if (!rmw_event_type_is_supported(type)) {
      GTEST_SKIP() << "deadline and liveliness events are not supported";
    }
  }
  auto node = std::make_shared<rclcpp::Node>("test_buffer_qos_events");
  const std::string topic("buffer_qos_events");
  const auto qos = rclcpp::QoS(10).reliable()
    .deadline(200ms)
    .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
    .liveliness_lease_duration(300ms);

  std::atomic<int> offered_deadline_missed{0};
  std::atomic<int> liveliness_lost{0};
  rclcpp::PublisherOptions pub_options;
  pub_options.event_callbacks.deadline_callback =
    [&](rclcpp::QOSDeadlineOfferedInfo &) {++offered_deadline_missed;};
  pub_options.event_callbacks.liveliness_callback =
    [&](rclcpp::QOSLivelinessLostInfo &) {++liveliness_lost;};

  std::atomic<int> requested_deadline_missed{0};
  std::atomic<int> alive_count{0};
  std::atomic<int> received{0};
  std::string received_backend;
  rclcpp::SubscriptionOptions sub_options;
  sub_options.acceptable_buffer_backends = "any";
  sub_options.event_callbacks.deadline_callback =
    [&](rclcpp::QOSDeadlineRequestedInfo &) {++requested_deadline_missed;};
  sub_options.event_callbacks.liveliness_callback =
    [&](rclcpp::QOSLivelinessChangedInfo & info) {alive_count = info.alive_count;};

  auto sub = node->create_subscription<ByteArray>(
    topic, qos,
    [&](const ByteArray & msg) {
      received_backend = msg.data.get_backend_type();
      ++received;
    }, sub_options);
  auto pub = node->create_publisher<ByteArray>(topic, qos, pub_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  RCPPUTILS_SCOPE_EXIT(
  {
    executor.cancel();
    spinner.join();
  });

  ASSERT_TRUE(wait_for([&]() {return pub->get_subscription_count() == 1u;}, 5s));

  // 20 Hz for 1 s, well within the deadline and the lease duration.
  for (int i = 0; i < 20; ++i) {
    ByteArray msg;
    msg.seq = static_cast<uint32_t>(i);
    msg.data = rosidl::Buffer<std::uint8_t>(
      std::make_unique<test_rosidl_buffer::TestBufferImpl<std::uint8_t>>(
        std::vector<std::uint8_t>(64, 1)));
    pub->publish(msg);
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_TRUE(wait_for([&]() {return received.load() > 0;}, 1s));
  const int alive_while_publishing = alive_count.load();

  // Stop publishing: deadlines are missed and liveliness is lost.
  auto all_events_reported = [&]() {
      return offered_deadline_missed.load() > 0 && requested_deadline_missed.load() > 0 &&
             liveliness_lost.load() > 0 && alive_count.load() == 0;
    };
  EXPECT_TRUE(wait_for(all_events_reported, 2s));

  EXPECT_EQ("test", received_backend);
  EXPECT_EQ(1, alive_while_publishing) << "the subscription never saw the publisher alive";
  EXPECT_GT(offered_deadline_missed.load(), 0) << "no offered deadline missed event";
  EXPECT_GT(requested_deadline_missed.load(), 0) << "no requested deadline missed event";
  EXPECT_GT(liveliness_lost.load(), 0) << "no liveliness lost event";
  EXPECT_EQ(0, alive_count.load()) << "the publisher is still alive after it stopped";
}
