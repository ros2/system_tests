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

#ifndef TEST_QUALITY_OF_SERVICE_QOS_UTILITIES_HPP
#define TEST_QUALITY_OF_SERVICE_QOS_UTILITIES_HPP

#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/// Helper time conversion method
/**
 * @param milliseconds
 * @return tuple of milliseconds converted to <seconds, nanoseconds>
 */
std::tuple<size_t, size_t> chrono_milliseconds_to_size_t(const std::chrono::milliseconds & milliseconds);

/// Simple publisher class used system tests
class Publisher : public rclcpp::Node
{
public:
    Publisher(std::string name,
      std::string topic,
      rclcpp::PublisherOptions<> pub_options,
      std::chrono::milliseconds publish_period,
      unsigned int max_publish_count = 0);

    ~Publisher();
    void start_publishing();

    /// name of this publisher
    std::string name_;
    /// topic name to publish
    std::string topic_;

    /// the timer of this publisher
    rclcpp::TimerBase::SharedPtr timer_ = nullptr;
    /// actual publishing mechanism
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = nullptr;

    rclcpp::PublisherOptions<> pub_options_;

    std::chrono::milliseconds publish_period_;

    /// number of messages published
    int sent_message_count_;
    /// max number of messages to publish
    int max_publish_count_;

};


class Subscriber : public rclcpp::Node
{
public:
  Subscriber(
    std::string name,
    std::string topic,
    rclcpp::SubscriptionOptions<> sub_options);

  int received_message_count_ = 0;
  int qos_event_count_ = 0;

private:
  std::string name_;
  std::string topic_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;
  rclcpp::SubscriptionOptions<> sub_options_;
};


#endif //TEST_QUALITY_OF_SERVICE_QOS_UTILITIES_HPP
