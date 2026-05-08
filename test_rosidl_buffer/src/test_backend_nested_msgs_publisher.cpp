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

// Publisher used by test_rosidl_buffer nested-message launch tests.

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "rosidl_buffer/buffer.hpp"
#include "test_rosidl_buffer/msg/byte_array_list.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

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

class TestBackendNestedMsgsPublisher : public rclcpp::Node
{
public:
  TestBackendNestedMsgsPublisher()
  : rclcpp::Node("test_backend_nested_msgs_publisher")
  {
    backend_mode_ = declare_parameter<std::string>("backend_mode", "cpu");
    const auto topic = declare_parameter<std::string>("topic_name", "test_byte_array_list");
    const auto rate_ms = declare_parameter<int>("publish_rate_ms", 100);
    max_count_ = static_cast<std::uint32_t>(declare_parameter<int>("max_publish_count", 50));

    if (backend_mode_ != "cpu" && backend_mode_ != "test") {
      RCLCPP_ERROR(
        get_logger(), "Invalid backend_mode '%s', falling back to 'cpu'", backend_mode_.c_str());
      backend_mode_ = "cpu";
    }

    test_rosidl_buffer::reset_to_cpu_call_count();

    pub_ = create_publisher<test_rosidl_buffer::msg::ByteArrayList>(topic, 10);
    count_pub_ = create_publisher<std_msgs::msg::UInt32>("publisher_count", 10);
    to_cpu_pub_ = create_publisher<std_msgs::msg::UInt32>("publisher_to_cpu_count", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(rate_ms),
      std::bind(&TestBackendNestedMsgsPublisher::on_timer, this));

    RCLCPP_INFO(
      get_logger(),
      "test_backend_nested_msgs_publisher started (mode=%s, topic=%s, max=%u)",
      backend_mode_.c_str(), topic.c_str(), max_count_);
  }

private:
  void on_timer()
  {
    if (max_count_ > 0 && seq_ >= max_count_) {
      return;
    }

    test_rosidl_buffer::msg::ByteArrayList msg;
    msg.items.resize(kItemCount);

    for (std::size_t item_index = 0; item_index < msg.items.size(); ++item_index) {
      auto & item = msg.items[item_index];
      item.seq = seq_;
      auto bytes = make_payload(seq_, item_index);

      if (backend_mode_ == "test") {
        auto impl = std::make_unique<test_rosidl_buffer::TestBufferImpl<std::uint8_t>>(
          std::move(bytes));
        item.data = rosidl::Buffer<std::uint8_t>(std::move(impl));
      } else {
        item.data = std::move(bytes);
      }
    }

    pub_->publish(msg);

    ++seq_;

    std_msgs::msg::UInt32 c;
    c.data = seq_;
    count_pub_->publish(c);

    std_msgs::msg::UInt32 tc;
    tc.data = static_cast<std::uint32_t>(
      test_rosidl_buffer::to_cpu_call_count().load(std::memory_order_relaxed));
    to_cpu_pub_->publish(tc);
  }

  std::string backend_mode_;
  std::uint32_t seq_ = 0;
  std::uint32_t max_count_ = 0;

  rclcpp::Publisher<test_rosidl_buffer::msg::ByteArrayList>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr count_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr to_cpu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestBackendNestedMsgsPublisher>());
  rclcpp::shutdown();
  return 0;
}
