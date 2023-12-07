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
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <tuple>

#include "gtest/gtest.h"

#include "rcutils/macros.h"
#include "rclcpp/rclcpp.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

/// Helper time conversion method
/**
 * @param milliseconds
 * @return tuple of milliseconds converted to <seconds, nanoseconds>
 */
std::tuple<size_t, size_t> convert_chrono_milliseconds_to_size_t(
  const std::chrono::milliseconds & milliseconds);

/// Helper wait for a predicate to evaluate to true
template<
  typename Predicate,
  typename TimeOutR, typename TimeOutP,
  typename PeriodR = int64_t, typename PeriodP = std::milli>
bool wait_for(
  Predicate predicate,
  const std::chrono::duration<TimeOutR, TimeOutP> & timeout,
  const std::chrono::duration<PeriodR, PeriodP> & period = std::chrono::milliseconds(100))
{
  auto end_time = std::chrono::steady_clock::now() + timeout;
  while (!predicate()) {
    if (std::chrono::steady_clock::now() > end_time) {
      return predicate();
    }
    std::this_thread::sleep_for(period);
  }
  return true;
}

class BaseQosRclcppTestFixture : public ::testing::Test
{
protected:
  void SetUp() override;
  void TearDown() override;
  // executor used to submit work
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  // empty promise used with dummy_future
  std::promise<bool> empty_promise;
  // a dummy to make executor timeout
  std::shared_future<bool> dummy_future;
  // test fixture publisher node
  std::shared_ptr<QosTestPublisher> publisher;
  // test fixture subscriber node
  std::shared_ptr<QosTestSubscriber> subscriber;
};

#define QosRclcppTestFixture CLASSNAME(QosRclcppTestFixture, RMW_IMPLEMENTATION)

class QosRclcppTestFixture : public BaseQosRclcppTestFixture
{
protected:
  const std::string this_rmw_implementation{RCUTILS_STRINGIFY(RMW_IMPLEMENTATION)};
};

#endif  // TEST_QUALITY_OF_SERVICE__QOS_UTILITIES_HPP_
