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

#ifndef TEST_QUALITY_OF_SERVICE__QOS_TEST_NODE_HPP_
#define TEST_QUALITY_OF_SERVICE__QOS_TEST_NODE_HPP_

#include <atomic>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "test_quality_of_service/visibility_control.hpp"

/// Base class used for QoS system test nodes
class QosTestNode : public rclcpp::Node
{
public:
  TEST_QUALITY_OF_SERVICE_PUBLIC
  QosTestNode(
    const std::string & name,
    const std::string & topic,
    const rclcpp::QoS & qos_options);

  TEST_QUALITY_OF_SERVICE_PUBLIC
  virtual ~QosTestNode();

  /// start working
  TEST_QUALITY_OF_SERVICE_PUBLIC
  void start();
  /// stop working
  TEST_QUALITY_OF_SERVICE_PUBLIC
  void stop();
  /// toggle between start and stop
  TEST_QUALITY_OF_SERVICE_PUBLIC
  void toggle();
  /// stop all work, should be called before destructor
  TEST_QUALITY_OF_SERVICE_PUBLIC
  virtual void teardown() = 0;
  /// return the number of measurements counted
  /**
   * \return the current count
   */
  TEST_QUALITY_OF_SERVICE_PUBLIC
  int get_count() const;

protected:
  TEST_QUALITY_OF_SERVICE_PUBLIC
  bool get_started() const;

  TEST_QUALITY_OF_SERVICE_PUBLIC
  virtual void setup_start() = 0;

  TEST_QUALITY_OF_SERVICE_PUBLIC
  virtual void setup_stop() = 0;

  /// a measurement was taken, increment the count
  /**
   * \return the incremented count
   */
  TEST_QUALITY_OF_SERVICE_PUBLIC
  int increment_count();

  /// name of this publisher (Node)
  const std::string name_;
  /// topic name to publish
  const std::string topic_;
  /// qos options needed for QoS settings
  const rclcpp::QoS qos_options_;

private:
  /// simple counter used for this node's measurements
  std::atomic<int> count_;
  /// true if started, false if otherwise
  bool started_;
  /// mutex used for starting, stopping, and toggling
  mutable std::recursive_mutex toggle_mutex_;
};

#endif  // TEST_QUALITY_OF_SERVICE__QOS_TEST_NODE_HPP_
