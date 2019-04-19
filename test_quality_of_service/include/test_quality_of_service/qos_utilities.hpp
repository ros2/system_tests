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

#ifndef TEST_QUALITY_OF_SERVICE__QOS_UTILITIES_HPP_
#define TEST_QUALITY_OF_SERVICE__QOS_UTILITIES_HPP_


#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <tuple>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


/// Helper time conversion method
/**
 * @param milliseconds
 * @return tuple of milliseconds converted to <seconds, nanoseconds>
 */
std::tuple<size_t, size_t> chrono_milliseconds_to_size_t(
  const std::chrono::milliseconds & milliseconds);

/// Simple publishing node used for system tests
class Publisher : public rclcpp::Node
{
public:
  Publisher(
    std::string name,
    std::string topic,
    rclcpp::PublisherOptions<> pub_options,
    std::chrono::milliseconds publish_period);

  ~Publisher();
  void start_publishing();
  void stop_publishing();
  void toggle_publishing();

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
  /// if true then currently has an active publishing timer
  bool is_publishing = false;
  std::recursive_mutex toggle_publish_mutex;
};

class Subscriber : public rclcpp::Node
{
public:
  Subscriber(
    std::string name,
    std::string topic,
    rclcpp::SubscriptionOptions<> sub_options);

  ~Subscriber();
  void start_listening();
  void stop_listening();
  void toggle_listening();

  int received_message_count_ = 0;
  bool is_listening = false;
  std::recursive_mutex toggle_subscriber_mutex;

private:
  std::string name_;
  std::string topic_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;
  rclcpp::SubscriptionOptions<> sub_options_;
};

/// Simple text fixture
class TestSetup : public ::testing::Test
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


#endif  // TEST_QUALITY_OF_SERVICE__QOS_UTILITIES_HPP_
