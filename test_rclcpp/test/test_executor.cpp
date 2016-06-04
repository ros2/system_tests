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

#include <atomic>
#include <cinttypes>
#include <future>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "test_rclcpp/utils.hpp"

#include "test_rclcpp/msg/u_int32.hpp"
#include "test_rclcpp/srv/add_two_ints.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif


TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), recursive_spin_call) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = rclcpp::Node::make_shared("recursive_spin_call");
  auto timer = node->create_wall_timer(0_s, [&executor]() {
    ASSERT_THROW(executor.spin_some(), std::runtime_error);
    ASSERT_THROW(executor.spin_once(), std::runtime_error);
    ASSERT_THROW(executor.spin(), std::runtime_error);
    executor.cancel();
  });
  executor.add_node(node);
  executor.spin();
}

TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), multithreaded_spin_call) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = rclcpp::Node::make_shared("multithreaded_spin_call");
  std::mutex m;
  bool ready = false;
  std::condition_variable cv;
  std::thread t([&executor, &m, &cv, &ready]() {
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [&ready] {return ready; });
    ASSERT_THROW(executor.spin_some(), std::runtime_error);
    ASSERT_THROW(executor.spin_once(), std::runtime_error);
    ASSERT_THROW(executor.spin(), std::runtime_error);
    executor.cancel();
  });
  auto timer = node->create_wall_timer(0_s, [&m, &cv, &ready]() {
    if (!ready) {
      {
        std::lock_guard<std::mutex> lock(m);
        ready = true;
      }
      cv.notify_one();
    }
  });
  executor.add_node(node);
  executor.spin();
  t.join();
}

// Try spinning 2 single-threaded executors in two separate threads.
TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), multiple_executors) {
  std::atomic_uint counter1;
  counter1 = 0;
  std::atomic_uint counter2;
  counter2 = 0;
  const uint32_t counter_goal = 20;

  // Initialize executor 1.
  rclcpp::executors::SingleThreadedExecutor executor1;
  auto callback1 = [&counter1, &counter_goal, &executor1]() {
      if (counter1 == counter_goal) {
        executor1.cancel();
        return;
      }
      ++counter1;
    };
  auto node1 = rclcpp::Node::make_shared("multiple_executors_1");
  auto timer1 = node1->create_wall_timer(1_ms, callback1);
  executor1.add_node(node1);

  // Initialize executor 2.
  rclcpp::executors::SingleThreadedExecutor executor2;

  auto callback2 = [&counter2, &counter_goal, &executor2]() {
      if (counter2 == counter_goal) {
        executor2.cancel();
        return;
      }
      ++counter2;
    };
  auto node2 = rclcpp::Node::make_shared("multiple_executors_2");
  auto timer2 = node2->create_wall_timer(1_ms, callback2);
  executor2.add_node(node2);

  auto spin_executor2 = [&executor2]() {
      executor2.spin();
    };

  // Launch both executors
  std::thread execution_thread(spin_executor2);

  executor1.spin();
  execution_thread.join();
  EXPECT_EQ(counter1.load(), counter_goal);
  EXPECT_EQ(counter2.load(), counter_goal);

  // Try to add node1 to executor2. It should throw, since node1 was already added to executor1.
  ASSERT_THROW(executor2.add_node(node1), std::runtime_error);
}

// Check that the executor is notified when a node adds a new timer, publisher, subscription,
// service or client.
TEST(CLASSNAME(test_executor, RMW_IMPLEMENTATION), notify) {
  rclcpp::executors::SingleThreadedExecutor executor;
  auto executor_spin_lambda = [&executor]() {
      executor.spin();
    };
  auto node = rclcpp::Node::make_shared("test_executor_notify");
  executor.add_node(node);
  {
    std::thread spin_thread(executor_spin_lambda);
    std::promise<void> timer_promise;
    std::shared_future<void> timer_future(timer_promise.get_future());

    auto timer = node->create_wall_timer(
      1_ms,
      [&timer_promise](rclcpp::TimerBase & timer)
    {
      timer_promise.set_value();
      timer.cancel();
    });
    EXPECT_EQ(std::future_status::ready, timer_future.wait_for(50_ms));
    executor.cancel();

    spin_thread.join();
  }

  {
    std::thread spin_thread(executor_spin_lambda);
    bool subscription_triggered = false;
    auto sub_callback =
      [&executor, &subscription_triggered](test_rclcpp::msg::UInt32::ConstSharedPtr msg) -> void
      {
        subscription_triggered = true;
        EXPECT_EQ(msg->data, 42);
        executor.cancel();
      };

    auto subscription = node->create_subscription<test_rclcpp::msg::UInt32>(
      "test_executor_notify_subscription",
      sub_callback,
      rmw_qos_profile_default);
    test_rclcpp::busy_wait_for_subscriber(node, "test_executor_notify_subscription");


    auto publisher = node->create_publisher<test_rclcpp::msg::UInt32>(
      "test_executor_notify_subscription", rmw_qos_profile_default);
    auto timer = node->create_wall_timer(
      1_ms,
      [&publisher]()
    {
      test_rclcpp::msg::UInt32 pub_msg;
      pub_msg.data = 42;
      publisher->publish(pub_msg);
    }
      );

    spin_thread.join();

    EXPECT_TRUE(subscription_triggered);
  }

  {
    std::thread spin_thread(executor_spin_lambda);

    auto service = node->create_service<test_rclcpp::srv::AddTwoInts>(
      "test_executor_notify_service",
      [](test_rclcpp::srv::AddTwoInts::Request::SharedPtr request,
      test_rclcpp::srv::AddTwoInts::Response::SharedPtr response)
    {
      response->sum = request->a + request->b;
    });

    auto client = node->create_client<test_rclcpp::srv::AddTwoInts>(
      "test_executor_notify_service"
      );
    auto request = std::make_shared<test_rclcpp::srv::AddTwoInts::Request>();
    request->a = 4;
    request->b = 2;
    auto future_result = client->async_send_request(request);
    EXPECT_EQ(future_result.get()->sum, 6);
    executor.cancel();
    spin_thread.join();
  }
}

// test removing a node

// test notify with multiple nodes

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
