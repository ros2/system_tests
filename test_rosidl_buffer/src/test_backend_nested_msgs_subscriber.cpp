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

// Subscriber used by test_rosidl_buffer nested-message launch tests.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/utilities.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "rosidl_buffer/buffer.hpp"
#include "test_rosidl_buffer/msg/byte_array_list.hpp"

namespace
{
constexpr std::size_t kItemCount = 3;
constexpr std::size_t kPayloadSize = 16;

std::vector<std::uint8_t> make_payload(std::uint32_t seq, std::size_t item_index)
{
  std::vector<std::uint8_t> bytes(kPayloadSize);
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    bytes[i] = static_cast<std::uint8_t>((seq + item_index * 17 + i) & 0xFF);
  }
  return bytes;
}
}  // namespace

class TestBackendNestedMsgsSubscriber : public rclcpp::Node
{
public:
  TestBackendNestedMsgsSubscriber()
  : rclcpp::Node("test_backend_nested_msgs_subscriber")
  {
    const auto topic = declare_parameter<std::string>("topic_name", "test_byte_array_list");
    const auto acceptable =
      declare_parameter<std::string>("acceptable_buffer_backends", "__unset__");

    rclcpp::SubscriptionOptions options;
    if (acceptable != "__unset__") {
      options.acceptable_buffer_backends = acceptable;
      RCLCPP_INFO(
        get_logger(), "acceptable_buffer_backends set to '%s'", acceptable.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "acceptable_buffer_backends: <default>");
    }

    sub_ = create_subscription<test_rosidl_buffer::msg::ByteArrayList>(
      topic, 10,
      std::bind(&TestBackendNestedMsgsSubscriber::on_msg, this, std::placeholders::_1),
      options);

    count_pub_ = create_publisher<std_msgs::msg::UInt32>("subscriber_count", 10);
    validation_pub_ = create_publisher<std_msgs::msg::Bool>("validation_result", 10);
    backend_pub_ = create_publisher<std_msgs::msg::String>("observed_backend_type", 10);

    RCLCPP_INFO(
      get_logger(),
      "test_backend_nested_msgs_subscriber started (topic=%s)",
      topic.c_str());
  }

private:
  void on_msg(const test_rosidl_buffer::msg::ByteArrayList::SharedPtr msg)
  {
    ++received_;
    bool ok = true;
    std::string backend = "empty";

    if (msg->items.size() != kItemCount) {
      RCLCPP_ERROR(
        get_logger(), "item count=%zu (expected %zu)", msg->items.size(), kItemCount);
      ok = false;
    }

    for (std::size_t item_index = 0; item_index < msg->items.size(); ++item_index) {
      const auto & item = msg->items[item_index];
      if (item_index == 0) {
        backend = item.data.get_backend_type();
      }

      if (item.data.size() != kPayloadSize) {
        RCLCPP_ERROR(
          get_logger(), "item=%zu seq=%u: size=%zu (expected %zu)",
          item_index, item.seq, item.data.size(), kPayloadSize);
        ok = false;
        continue;
      }

      const auto expected = make_payload(item.seq, item_index);
      const auto bytes = item.data.to_vector();
      for (std::size_t i = 0; i < kPayloadSize; ++i) {
        if (bytes[i] != expected[i]) {
          RCLCPP_ERROR(
            get_logger(), "item=%zu seq=%u: byte[%zu]=%u (expected %u)",
            item_index, item.seq, i, bytes[i], expected[i]);
          ok = false;
          break;
        }
      }
    }

    validation_ = validation_ && ok;

    std_msgs::msg::UInt32 c;
    c.data = received_;
    count_pub_->publish(c);

    std_msgs::msg::Bool v;
    v.data = validation_;
    validation_pub_->publish(v);

    std_msgs::msg::String b;
    b.data = backend;
    backend_pub_->publish(b);
  }

  std::uint32_t received_ = 0;
  bool validation_ = true;

  rclcpp::Subscription<test_rosidl_buffer::msg::ByteArrayList>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr backend_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestBackendNestedMsgsSubscriber>());
  rclcpp::shutdown();
  return 0;
}
