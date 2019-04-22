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
  /// switch publishing state
  void toggle_publishing();

  /// name of this publisher (Node)
  std::string name_;

  /// topic name to publish
  std::string topic_;

  /// the timer of this publisher
  rclcpp::TimerBase::SharedPtr timer_ = nullptr;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = nullptr;

  /// publisher options needed for QoS settings
  rclcpp::PublisherOptions<> pub_options_;

  /// publishing period
  std::chrono::milliseconds publish_period_;

  /// number of messages published
  int sent_message_count_ = 0;

  /// if true then currently has an active publishing timer
  bool is_publishing = false;

  /// mutex used for starting, stopping, and toggling publishing
  std::recursive_mutex toggle_publish_mutex;
};

/// Simple subscriber node used for system tests
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
  /// switch subscription state
  void toggle_listening();

  /// number of messages received
  int received_message_count_ = 0;

  /// if true then actively listening for subscribed messages
  bool is_listening = false;

  /// mutex used for starting, stopping, and toggling subscription
  std::recursive_mutex toggle_subscriber_mutex;

  /// the name of this subscriber (Node)
  std::string name_;

  /// desired topic for subscription
  std::string topic_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_ = nullptr;

  /// subscription options needed for QoS settings
  rclcpp::SubscriptionOptions<> sub_options_;
};

/// Simple text fixture: standard bring-up and teardown for tests
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
  /// executor used to submit work
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
};


#endif  // TEST_QUALITY_OF_SERVICE__QOS_UTILITIES_HPP_
