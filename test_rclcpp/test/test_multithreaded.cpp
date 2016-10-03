// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include <limits>
#include <memory>
#include <string>
#include <thread>  // TODO(wjwwood): remove me when Connext and FastRTPS exclusions are removed
#include <utility>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/exceptions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "test_rclcpp/utils.hpp"
#include "test_rclcpp/msg/u_int32.hpp"
#include "test_rclcpp/srv/add_two_ints.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

static inline void multi_consumer_pub_sub_test(bool intra_process)
{
  std::string node_topic_name = "multi_consumer";
  if (intra_process) {
    node_topic_name += "_intra_process";
  }

  auto node = rclcpp::Node::make_shared(node_topic_name, intra_process);
  auto callback_group = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto pub = node->create_publisher<test_rclcpp::msg::UInt32>(node_topic_name, 16);

  std::vector<rclcpp::Subscription<test_rclcpp::msg::UInt32>::SharedPtr> subscriptions;
  std::atomic_int counter(0);

  auto callback =
    [&counter, &intra_process](test_rclcpp::msg::UInt32::ConstSharedPtr msg,
      const rmw_message_info_t & info) -> void
    {
      counter.fetch_add(1);
      printf("callback() %d with message data %u\n", counter.load(), msg->data);
      ASSERT_EQ(intra_process, info.from_intra_process);
    };

  rclcpp::executors::MultiThreadedExecutor executor;
  // Try to saturate the MultithreadedExecutor's thread pool with subscriptions
  for (uint32_t i = 0; i < executor.get_number_of_threads(); i++) {
    auto sub = node->create_subscription<test_rclcpp::msg::UInt32>(node_topic_name, 16, callback,
        callback_group);
    subscriptions.push_back(sub);
  }
  ASSERT_TRUE(std::numeric_limits<int>::max() > subscriptions.size());
  int subscriptions_size = static_cast<int>(subscriptions.size());

  executor.add_node(node);
  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  msg->data = 0;

  // wait a moment for everything to initialize
  test_rclcpp::busy_wait_for_subscriber(node, "multi_consumer");

  // sanity check that no callbacks have fired
  EXPECT_EQ(0, counter.load());

  ++msg->data;
  pub->publish(msg);

  // test spin_some
  // Expectation: The message was published and all subscriptions fired the callback.
  // Use spin_once to block until published message triggers an event
  // Note that we have no guarantee that spinning once will deliver all
  // messages. So we put a heuristic upper bound (2s) on how long we're willing to
  // wait for delivery to occur.
  const std::chrono::milliseconds sleep_per_loop(10);
  const unsigned int max_loops = 200;
  std::atomic_uint loop(0);
  while ((counter.load() != subscriptions_size) && (loop++ < max_loops)) {
    rclcpp::utilities::sleep_for(sleep_per_loop);
    executor.spin_some();
  }
  EXPECT_EQ(counter.load(), subscriptions_size);

  // Expectation: no further messages were received.
  executor.spin_once(std::chrono::milliseconds(0));
  EXPECT_EQ(counter.load(), subscriptions_size);

  // reset counter
  counter.store(0);
  msg->data = 0;

  ASSERT_TRUE(std::numeric_limits<int>::max() > (5 * subscriptions.size()));
  const int expected_count = static_cast<int>(5 * subscriptions.size());

  std::mutex publish_mutex;
  auto publish_callback = [
    msg, &pub, &executor, &counter, &expected_count, &sleep_per_loop, &max_loops, &publish_mutex](
    rclcpp::timer::TimerBase & timer) -> void
    {
      std::lock_guard<std::mutex> lock(publish_mutex);
      ++msg->data;
      if (msg->data > 5) {
        timer.cancel();
        // wait for the last callback to fire before cancelling
        // Wait for pending subscription callbacks to trigger.
        std::atomic_uint loop(0);
        while ((counter.load() != expected_count) && (loop++ < max_loops)) {
          std::this_thread::sleep_for(sleep_per_loop);
        }
        executor.cancel();
        return;
      }
      pub->publish(msg);
    };
  auto timer = node->create_wall_timer(std::chrono::milliseconds(1), publish_callback);

  executor.spin();
  EXPECT_EQ(counter.load(), expected_count);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_consumer_single_producer) {
  // multiple subscriptions, single publisher
  multi_consumer_pub_sub_test(false);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_consumer_intra_process) {
  // multiple subscriptions, single publisher, intra-process
  multi_consumer_pub_sub_test(true);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_consumer_clients) {
  // multiple clients, single server
  auto node = rclcpp::Node::make_shared("multi_consumer_clients");
  rclcpp::executors::MultiThreadedExecutor executor;

  std::atomic_int counter(0);
  auto callback = [&counter](const std::shared_ptr<test_rclcpp::srv::AddTwoInts::Request> request,
      std::shared_ptr<test_rclcpp::srv::AddTwoInts::Response> response)
    {
      printf("Called service callback: %d\n", counter.load());
      ++counter;
      response->sum = request->a + request->b;
    };

  rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
  qos_profile.depth = executor.get_number_of_threads() * 2;
  auto callback_group = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto service = node->create_service<test_rclcpp::srv::AddTwoInts>(
    "multi_consumer_clients", callback, qos_profile, callback_group);

  using ClientRequestPair =
      std::pair<rclcpp::client::Client<test_rclcpp::srv::AddTwoInts>::SharedPtr,
      test_rclcpp::srv::AddTwoInts::Request::SharedPtr>;
  using SharedFuture = rclcpp::client::Client<test_rclcpp::srv::AddTwoInts>::SharedFuture;


  std::vector<ClientRequestPair> client_request_pairs;
  for (uint32_t i = 0; i < 2 * executor.get_number_of_threads(); ++i) {
    auto client = node->create_client<test_rclcpp::srv::AddTwoInts>(
      "multi_consumer_clients", qos_profile, callback_group);
    auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
    request->a = i;
    request->b = i + 1;
    client_request_pairs.push_back(ClientRequestPair(client, request));
  }
  ASSERT_TRUE(std::numeric_limits<int>::max() > client_request_pairs.size());
  int client_request_pairs_size = static_cast<int>(client_request_pairs.size());

  executor.add_node(node);
  rclcpp::utilities::sleep_for(5_ms);

  executor.spin_some();
  // No callbacks should have fired
  EXPECT_EQ(0, counter.load());

  {
    std::vector<SharedFuture> results;
    // Send all the requests
    for (auto & pair : client_request_pairs) {
      {  // TODO(wjwwood): remove this block when Connext and FastRTPS support wait_for_service.
        try {
          if (!pair.first->wait_for_service(20_s)) {
            ASSERT_TRUE(false) << "service not available after waiting";
          }
        } catch (rclcpp::exceptions::RCLError) {
          std::this_thread::sleep_for(1_s);
        }
      }
      results.push_back(pair.first->async_send_request(pair.second));
    }
    // Wait on the future produced by the first request
    auto result = executor.spin_until_future_complete(results.back());

    ASSERT_EQ(result, rclcpp::executor::FutureReturnCode::SUCCESS);

    // Check the status of all futures
    for (uint32_t i = 0; i < results.size(); i++) {
      ASSERT_EQ(std::future_status::ready, results[i].wait_for(std::chrono::seconds(0)));
      EXPECT_EQ(results[i].get()->sum, 2 * i + 1);
    }

    EXPECT_EQ(counter.load(), client_request_pairs_size);
  }

  // Reset the counter and try again with spin
  counter = 0;
  {
    std::vector<SharedFuture> results;
    // Send all the requests again
    for (auto & pair : client_request_pairs) {
      {  // TODO(wjwwood): remove this block when Connext and FastRTPS support wait_for_service.
        try {
          if (!pair.first->wait_for_service(20_s)) {
            ASSERT_TRUE(false) << "service not available after waiting";
          }
        } catch (rclcpp::exceptions::RCLError) {
          std::this_thread::sleep_for(1_s);
        }
      }
      results.push_back(pair.first->async_send_request(pair.second));
    }
    auto timer_callback = [&executor, &results]() {
        for (auto & result : results) {
          if (result.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
            return;
          }
        }
        executor.cancel();
      };
    auto timer = node->create_wall_timer(std::chrono::milliseconds(3), timer_callback);

    executor.spin();

    // Check the status of all futures
    for (uint32_t i = 0; i < results.size(); i++) {
      ASSERT_EQ(std::future_status::ready, results[i].wait_for(std::chrono::seconds(0)));
      EXPECT_EQ(results[i].get()->sum, 2 * i + 1);
    }
    EXPECT_EQ(counter.load(), client_request_pairs_size);
  }
}

static inline void multi_access_publisher(bool intra_process)
{
  // Try to access the same publisher simultaneously
  auto context = std::make_shared<rclcpp::context::Context>();
  std::string node_topic_name = "multi_access_publisher";
  if (intra_process) {
    node_topic_name += "_intra_process";
  }

  auto node = rclcpp::Node::make_shared(node_topic_name, context, intra_process);
  auto timer_callback_group = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::Reentrant);
  auto sub_callback_group = node->create_callback_group(
    rclcpp::callback_group::CallbackGroupType::Reentrant);

  rclcpp::executors::MultiThreadedExecutor executor;

  const size_t num_messages = 5 * executor.get_number_of_threads();
  auto pub = node->create_publisher<test_rclcpp::msg::UInt32>(node_topic_name, num_messages);
  // callback groups?
  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();
  // use atomic
  std::atomic_uint timer_counter(0);
  std::atomic_uint subscription_counter(0);

  auto timer_callback =
    [&executor, &pub, &msg, &timer_counter, &subscription_counter, &num_messages](
    rclcpp::timer::TimerBase & timer)
    {
      if (timer_counter.load() >= num_messages) {
        timer.cancel();
        std::atomic_uint i(0);
        // Wait for pending subscription callbacks to trigger.
        while (subscription_counter < timer_counter &&
          ++i <= num_messages)
        {
          rclcpp::utilities::sleep_for(1_ms);
        }
        executor.cancel();
        return;
      }
      msg->data = ++timer_counter;
      printf("Publishing message %u\n", timer_counter.load());
      pub->publish(msg);
    };
  std::vector<rclcpp::timer::TimerBase::SharedPtr> timers;
  // timers will fire simultaneously in each thread
  for (uint32_t i = 0; i < executor.get_number_of_threads(); i++) {
    timers.push_back(node->create_wall_timer(std::chrono::milliseconds(1), timer_callback));
  }

  auto sub_callback = [&subscription_counter](const test_rclcpp::msg::UInt32::SharedPtr)
    {
      ++subscription_counter;
      printf("Subscription callback %u\n", subscription_counter.load());
    };
  auto sub = node->create_subscription<test_rclcpp::msg::UInt32>(node_topic_name, sub_callback,
      rmw_qos_profile_default,
      sub_callback_group);
  executor.add_node(node);
  executor.spin();
  ASSERT_EQ(timer_counter.load(), num_messages);
  ASSERT_EQ(timer_counter.load(), subscription_counter);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_access_publisher) {
  multi_access_publisher(false);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_access_publisher_intra_process) {
  multi_access_publisher(true);
}


int main(int argc, char ** argv)
{
  // NOTE: use custom main to ensure that rclcpp::init is called only once
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
