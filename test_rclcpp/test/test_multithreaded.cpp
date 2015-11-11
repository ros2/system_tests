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

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "test_rclcpp/msg/u_int32.hpp"

#ifdef RMW_IMPLEMENTATION
# define CLASSNAME_(NAME, SUFFIX) NAME ## __ ## SUFFIX
# define CLASSNAME(NAME, SUFFIX) CLASSNAME_(NAME, SUFFIX)
#else
# define CLASSNAME(NAME, SUFFIX) NAME
#endif

// TODO test for utilization on all cores? maybe just test time?

inline void multi_consumer_pub_sub_test(bool intra_process) {
  rclcpp::init(0, nullptr);

  auto node = rclcpp::Node::make_shared("test_multithreaded", intra_process);
  auto pub = node->create_publisher<test_rclcpp::msg::UInt32>("test_multithreaded", 10);

  std::vector<rclcpp::Subscription<test_rclcpp::msg::UInt32>::SharedPtr> subscriptions;
  std::size_t counter = 0;

  auto callback =
    [&counter, &intra_process](test_rclcpp::msg::UInt32::ConstSharedPtr msg,
      const rmw_message_info_t & info) -> void
    {
      ++counter;
      printf("callback() %4u with message data %u\n", counter, msg->data);
      ASSERT_EQ(intra_process, info.from_intra_process);
    };

  rclcpp::executors::MultiThreadedExecutor executor;
  // Try to saturate the MultithreadedExecutor's thread pool with subscriptions
  for (std::size_t i = 0; i < 2*executor.get_number_of_threads(); i++) {
    auto sub = node->create_subscription<test_rclcpp::msg::UInt32>("test_multithreaded", 10, callback);
    subscriptions.push_back(sub);
  }

  executor.add_node(node);
  auto msg = std::make_shared<test_rclcpp::msg::UInt32>();

  // wait a moment for everything to initialize (TODO: fix nondeterministic startup behavior)
  rclcpp::utilities::sleep_for(1_ms);

  // test spin_once
  executor.spin_once();
  ASSERT_EQ(0, counter);
  ++msg->data;
  pub->publish(msg);

  // test spin_some
  // Expectation: The message was published and some number of subscriptions have fired the callback.
  // Should we expect all subscriptions to be have been serviced?
  executor.spin_some();
  ASSERT_EQ(counter, subscriptions.size());

  counter = 0;

  ++msg->data;
  pub->publish(msg);

  // What's the expected behavior after spin_once?
  // Should rclcpp execute one executable? (expect counter == 1)
  // Or should work be distributed to each thread, and each thread executes one executable (expect counter == # of threads);
  for (std::size_t i = 1; i <= subscriptions.size(); i++) {
    executor.spin_once();
    ASSERT_EQ(i, counter);
  }
  // We don't expect any more callbacks once all subscriptions were serviced
  executor.spin_once();
  ASSERT_EQ(subscriptions.size(), counter);

  // reset counter
  counter = 0;

  std::thread publish_thread(
    [&pub, &msg]() {
      // sleep and publish 10 times, then quit
      for (std::size_t i = 1; i <= 10; i++) {
        msg->data = i;
        pub->publish(msg);
        rclcpp::utilities::sleep_for(1_ms);
      }
      rclcpp::shutdown();
    }
  );

  // test spin (with timeout controlled by publish_thread)
  executor.spin();
  publish_thread.join();
  ASSERT_EQ(counter, 10*subscriptions.size());
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_consumer_single_producer) {
  // multiple subscriptions, single publisher
  multi_consumer_pub_sub_test(false);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_consumer_intraprocess) {
  // multiple subscriptions, single publisher, intra-process
  multi_consumer_pub_sub_test(true);
}

TEST(CLASSNAME(test_multithreaded, RMW_IMPLEMENTATION), multi_consumer_clients) {
  // multiple clients, single server
  auto node = rclcpp::Node::make_shared("test_multithreaded");
  // TODO 
  //auto service = node->


  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
}

