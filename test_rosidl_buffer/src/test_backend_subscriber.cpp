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

// Subscriber used by test_rosidl_buffer launch tests.
//
// Parameters:
//   topic_name                  : std topic (default "test_bytes")
//   expected_backend            : required backend_type reported on the wire
//                                 (e.g. "test" or "cpu")
//   acceptable_buffer_backends  : value for SubscriptionOptions; leave as the
//                                 sentinel "__unset__" to use defaults
//                                 (this is what the test-to-cpu scenario needs)
//
// Side channels published for the launch-test harness:
//   /subscriber_count        (std_msgs/UInt32)  — callbacks observed so far
//   /validation_result       (std_msgs/Bool)    — cumulative pass/fail
//   /observed_backend_type   (std_msgs/String)  — backend of most recent msg

#include <cstdint>
#include <memory>
#include <string>

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
#include "test_rosidl_buffer/msg/byte_array.hpp"

namespace
{
constexpr std::size_t kPayloadSize = 64;
}

class TestBackendSubscriber : public rclcpp::Node
{
public:
  TestBackendSubscriber()
  : rclcpp::Node("test_backend_subscriber")
  {
    const auto topic = declare_parameter<std::string>("topic_name", "test_bytes");
    expected_backend_ = declare_parameter<std::string>("expected_backend", "test");
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

    sub_ = create_subscription<test_rosidl_buffer::msg::ByteArray>(
      topic, 10,
      std::bind(&TestBackendSubscriber::on_msg, this, std::placeholders::_1),
      options);

    count_pub_ = create_publisher<std_msgs::msg::UInt32>("subscriber_count", 10);
    validation_pub_ = create_publisher<std_msgs::msg::Bool>("validation_result", 10);
    backend_pub_ = create_publisher<std_msgs::msg::String>("observed_backend_type", 10);

    RCLCPP_INFO(
      get_logger(),
      "test_backend_subscriber started (topic=%s, expected_backend=%s)",
      topic.c_str(), expected_backend_.c_str());
  }

private:
  void on_msg(const test_rosidl_buffer::msg::ByteArray::SharedPtr msg)
  {
    ++received_;
    bool ok = true;

    const std::string backend = msg->data.get_backend_type();
    if (backend != expected_backend_) {
      RCLCPP_ERROR(
        get_logger(), "seq=%u: backend '%s' != expected '%s'",
        msg->seq, backend.c_str(), expected_backend_.c_str());
      ok = false;
    }

    if (msg->data.size() != kPayloadSize) {
      RCLCPP_ERROR(
        get_logger(), "seq=%u: size=%zu (expected %zu)",
        msg->seq, msg->data.size(), kPayloadSize);
      ok = false;
    } else {
      const auto bytes = msg->data.to_vector();
      for (std::size_t i = 0; i < kPayloadSize; ++i) {
        const auto expected = static_cast<std::uint8_t>((msg->seq + i) & 0xFF);
        if (bytes[i] != expected) {
          RCLCPP_ERROR(
            get_logger(), "seq=%u: byte[%zu]=%u (expected %u)",
            msg->seq, i, bytes[i], expected);
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

  std::string expected_backend_;
  std::uint32_t received_ = 0;
  bool validation_ = true;

  rclcpp::Subscription<test_rosidl_buffer::msg::ByteArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr validation_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr backend_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestBackendSubscriber>());
  rclcpp::shutdown();
  return 0;
}
