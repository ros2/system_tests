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

// An accelerated subscription and a buffer-aware publisher in the same process
// share the accelerated channel topic `rt/<topic>/_buf/<subscription gid>`.
// Check that the topic is deleted once both are destroyed, whatever the order,
// so that it does not leak and the participant can be deleted on shutdown.

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "rcl/node.h"
#include "rclcpp/rclcpp.hpp"
#include "rmw_fastrtps_cpp/get_participant.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

using namespace std::chrono_literals;
using ByteArray = test_rosidl_buffer::msg::ByteArray;

class TestAccelTopicLifetime : public ::testing::Test
{
protected:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_accel_topic_lifetime");
    participant_ = rmw_fastrtps_cpp::get_domain_participant(
      rcl_node_get_rmw_handle(node_->get_node_base_interface()->get_rcl_node_handle()));
    ASSERT_NE(nullptr, participant_);
  }

  void TearDown() override
  {
    node_.reset();
    rclcpp::shutdown();
  }

  // Creates a publisher and an accelerated subscription on `topic`, and waits until
  // a message has been delivered through the accelerated channel.
  // Returns the name of the accelerated channel topic.
  std::string create_accelerated_pair(const std::string & topic)
  {
    std::string backend;
    rclcpp::SubscriptionOptions options;
    options.acceptable_buffer_backends = "any";
    sub_ = node_->create_subscription<ByteArray>(
      topic, 10, [&backend](const ByteArray & msg) {backend = msg.data.get_backend_type();},
      options);
    pub_ = node_->create_publisher<ByteArray>(topic, 10);

    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (backend != "test" && std::chrono::steady_clock::now() < deadline) {
      ByteArray msg;
      auto impl = std::make_unique<test_rosidl_buffer::TestBufferImpl<std::uint8_t>>(
        std::vector<std::uint8_t>(64, 1));
      msg.data = rosidl::Buffer<std::uint8_t>(std::move(impl));
      pub_->publish(msg);
      rclcpp::spin_some(node_);
      std::this_thread::sleep_for(50ms);
    }
    EXPECT_EQ("test", backend) << "no message received through the accelerated channel";

    auto infos = node_->get_subscriptions_info_by_topic(topic);
    EXPECT_EQ(1u, infos.size());
    if (infos.empty()) {
      return "";
    }
    static const char hex_chars[] = "0123456789abcdef";
    std::string gid_hex;
    for (std::uint8_t byte : infos[0].endpoint_gid()) {
      gid_hex += hex_chars[(byte >> 4) & 0xF];
      gid_hex += hex_chars[byte & 0xF];
    }
    return "rt/" + topic + "/_buf/" + gid_hex;
  }

  rclcpp::Node::SharedPtr node_;
  eprosima::fastdds::dds::DomainParticipant * participant_{nullptr};
  rclcpp::Publisher<ByteArray>::SharedPtr pub_;
  rclcpp::Subscription<ByteArray>::SharedPtr sub_;
};

TEST_F(TestAccelTopicLifetime, subscription_destroyed_first) {
  const std::string accel_topic = create_accelerated_pair("accel_topic_lifetime_sub_first");
  ASSERT_NE(nullptr, participant_->lookup_topicdescription(accel_topic)) << accel_topic;

  sub_.reset();
  pub_.reset();
  EXPECT_EQ(nullptr, participant_->lookup_topicdescription(accel_topic))
    << accel_topic << " leaked";
}

TEST_F(TestAccelTopicLifetime, publisher_destroyed_first) {
  const std::string accel_topic = create_accelerated_pair("accel_topic_lifetime_pub_first");
  ASSERT_NE(nullptr, participant_->lookup_topicdescription(accel_topic)) << accel_topic;

  pub_.reset();
  sub_.reset();
  EXPECT_EQ(nullptr, participant_->lookup_topicdescription(accel_topic))
    << accel_topic << " leaked";
}
