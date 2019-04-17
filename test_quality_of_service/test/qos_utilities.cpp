// Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "test_quality_of_service/qos_utilities.hpp"

using namespace std::chrono_literals;


std::tuple<size_t, size_t> chrono_milliseconds_to_size_t(const std::chrono::milliseconds & milliseconds)
{
  size_t seconds = milliseconds.count() / 1000;
  size_t nanoseconds = (milliseconds.count() % 1000) * 1000000;
  return std::make_tuple(seconds, nanoseconds);
}

Publisher::Publisher(
  const std::string name,
  const std::string topic,
  rclcpp::PublisherOptions<> pub_options,
  std::chrono::milliseconds publish_period,
  uint32_t max_publish_count)
  : Node(name),
  name_(name),
  topic_(topic),
  pub_options_(pub_options),
  publish_period_(publish_period),
  max_publish_count_(max_publish_count)
{
  RCLCPP_INFO(this->get_logger(), "created publisher %s %s\n", name.c_str(), topic.c_str());
  /// create publisher for given topic name
}

void Publisher::start_publishing() {

  publisher_ = this->create_publisher<std_msgs::msg::String>(topic_, pub_options_);

  auto timer_callback =
    [this]() -> void {
        auto message = std_msgs::msg::String();

        message.data = this->name_ + ": testing " + std::to_string(this->sent_message_count_++);

        RCLCPP_INFO(this->get_logger(), this->name_ + ": publishing: '%s'", message.data.c_str());

        this->publisher_->publish(message);

        if(this->max_publish_count_ > 0 && this->sent_message_count_ >= this->max_publish_count_) {
          RCLCPP_INFO(this->get_logger(), "max_publish_count_ reached, halting publishing");
          this->timer_->cancel();
        }
    };
  timer_ = this->create_wall_timer(publish_period_, timer_callback);
}


//todo stop publishing

Publisher::~Publisher() {
  if(timer_) {
    timer_->cancel();
  }
  if(publisher_) {
    publisher_.reset();
  }
}

Subscriber::Subscriber(
  const std::string name,
  const std::string topic,
  rclcpp::SubscriptionOptions<> sub_options)
  : Node(name),
  name_(name),
  topic_(topic)
{
  RCLCPP_INFO(get_logger(), "created subscriber %s %s", name.c_str(), topic.c_str());

  subscription_ = create_subscription<std_msgs::msg::String>(
    topic_,
    [this](const typename std_msgs::msg::String::SharedPtr msg) -> void
    {
        RCLCPP_INFO(get_logger(), this->name_ + ": subscriber heard: [%s]", msg->data.c_str());
        this->received_message_count_++;
    },
    sub_options);
}
