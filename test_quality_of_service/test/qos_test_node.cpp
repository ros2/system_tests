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

#include <atomic>
#include <mutex>
#include <string>

#include "test_quality_of_service/qos_test_node.hpp"

#include "rclcpp/rclcpp.hpp"

QosTestNode::QosTestNode(
  const std::string & name,
  const std::string & topic)
: Node(name),
  name_(topic),
  topic_(topic),
  count_(0)
{}

QosTestNode::~QosTestNode()
{}

int QosTestNode::get_count()
{
  return count_.load();
}

int QosTestNode::increment_count()
{
  count_++;
  return count_.load();
}

bool QosTestNode::get_started()
{
  return started_;
}

void QosTestNode::toggle()
{
  std::unique_lock<std::recursive_mutex> ulock {toggle_mutex_};
  if (started_) {
    stop();
  } else {
    start();
  }
}

void QosTestNode::start()
{
  std::unique_lock<std::recursive_mutex> ulock {toggle_mutex_};
  if (started_) {
    return;
  }
  started_ = true;
  setup_start();
}

void QosTestNode::stop()
{
  std::unique_lock<std::recursive_mutex> ulock {toggle_mutex_};
  if (!started_) {
    return;
  }
  started_ = false;
  setup_stop();
}
