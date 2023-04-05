// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "gtest/gtest.h"

#include "rcl/graph.h"

#include "rclcpp/rclcpp.hpp"

#include "test_msgs/srv/empty.hpp"

#include "test_quality_of_service/qos_test_publisher.hpp"
#include "test_quality_of_service/qos_test_subscriber.hpp"
#include "test_quality_of_service/qos_utilities.hpp"

using namespace std::chrono_literals;

TEST_F(QosRclcppTestFixture, test_best_available_policies_subscription) {
  const std::string topic = "/test_best_available_subscription";
  const std::chrono::milliseconds publish_period{5000};
  const std::chrono::milliseconds publisher_deadline{2};
  const std::chrono::milliseconds publisher_liveliness_lease_duration{3};

  rclcpp::QoS publisher_qos_profile(1);
  publisher_qos_profile
  .best_effort()
  .transient_local()
  .deadline(publisher_deadline)
  .liveliness(rclcpp::LivelinessPolicy::Automatic)
  .liveliness_lease_duration(publisher_liveliness_lease_duration);

  rclcpp::BestAvailableQoS subscription_qos_profile;

  publisher = std::make_shared<QosTestPublisher>(
    "publisher", topic, publisher_qos_profile, publish_period);
  publisher->toggle();

  subscriber = std::make_shared<QosTestSubscriber>(
    "subscriber", topic, subscription_qos_profile);

  // Wait for discovery before creating subscription
  auto allocator = subscriber->get_node_options().allocator();
  bool discovery_successful = false;
  rcl_ret_t rcl_ret = rcl_wait_for_publishers(
    subscriber->get_node_base_interface()->get_rcl_node_handle(),
    &allocator,
    topic.c_str(),
    1u,
    static_cast<rcutils_duration_value_t>(5e9),  // 5 second timeout
    &discovery_successful);
  ASSERT_EQ(rcl_ret, RCL_RET_OK) << rcl_get_error_string().str;
  ASSERT_TRUE(discovery_successful);

  subscriber->toggle();

  // Check actual subscription QoS
  // We expect it to have exactly the same policies as the publisher for some subset of policies
  std::vector<rclcpp::TopicEndpointInfo> subscriptions_info;
  // Here we wait for the subscription created during the "toggle" above to appear, which may take
  // some time.
  bool wait_ret = ::wait_for(
    [this, &topic, &subscriptions_info]() {
      subscriptions_info = subscriber->get_subscriptions_info_by_topic(topic);
      return subscriptions_info.size() == 1u;
    }, 5s);
  ASSERT_TRUE(wait_ret);
  ASSERT_EQ(subscriptions_info.size(), 1u);
  const auto & actual_qos = subscriptions_info[0].qos_profile();
  EXPECT_EQ(actual_qos.reliability(), publisher_qos_profile.reliability());
  EXPECT_EQ(actual_qos.durability(), publisher_qos_profile.durability());
  EXPECT_EQ(actual_qos.deadline(), publisher_qos_profile.deadline());
  EXPECT_EQ(actual_qos.liveliness(), publisher_qos_profile.liveliness());
  EXPECT_EQ(
    actual_qos.liveliness_lease_duration(), publisher_qos_profile.liveliness_lease_duration());
}

TEST_F(QosRclcppTestFixture, test_best_available_policies_publisher) {
  const std::string topic = "/test_best_available_publisher";
  const std::chrono::milliseconds publish_period{5000};
  const std::chrono::milliseconds subscription_deadline{2};
  const std::chrono::milliseconds subscription_liveliness_lease_duration{3};

  rclcpp::QoS subscription_qos_profile(1);
  subscription_qos_profile
  .best_effort()
  .durability_volatile()
  .deadline(subscription_deadline)
  .liveliness(rclcpp::LivelinessPolicy::ManualByTopic)
  .liveliness_lease_duration(subscription_liveliness_lease_duration);

  rclcpp::BestAvailableQoS publisher_qos_profile;

  publisher = std::make_shared<QosTestPublisher>(
    "publisher", topic, publisher_qos_profile, publish_period);

  subscriber = std::make_shared<QosTestSubscriber>(
    "subscriber", topic, subscription_qos_profile);
  subscriber->toggle();

  // Wait for discovery before creating publisher
  auto allocator = publisher->get_node_options().allocator();
  bool discovery_successful = false;
  rcl_ret_t rcl_ret = rcl_wait_for_subscribers(
    publisher->get_node_base_interface()->get_rcl_node_handle(),
    &allocator,
    topic.c_str(),
    1u,
    static_cast<rcutils_duration_value_t>(5e9),  // 5 second timeout
    &discovery_successful);
  ASSERT_EQ(rcl_ret, RCL_RET_OK) << rcl_get_error_string().str;
  ASSERT_TRUE(discovery_successful);

  publisher->toggle();

  // Check actual publisher QoS
  // We expect it to have exactly the same policies as the publisher for some subset of policies
  // However, it should always be reliable and transient local (for DDS middlewares)
  std::vector<rclcpp::TopicEndpointInfo> publishers_info;
  // Here we wait for the publisher created during the "toggle" above to appear, which may take
  // some time.
  bool wait_ret = ::wait_for(
    [this, &topic, &publishers_info]() {
      publishers_info = publisher->get_publishers_info_by_topic(topic);
      return publishers_info.size() == 1u;
    }, 5s);
  ASSERT_TRUE(wait_ret);
  ASSERT_EQ(publishers_info.size(), 1u);
  const auto & actual_qos = publishers_info[0].qos_profile();
  EXPECT_EQ(actual_qos.reliability(), rclcpp::ReliabilityPolicy::Reliable);
  EXPECT_EQ(actual_qos.durability(), rclcpp::DurabilityPolicy::TransientLocal);
  EXPECT_EQ(actual_qos.deadline(), subscription_qos_profile.deadline());
  EXPECT_EQ(actual_qos.liveliness(), subscription_qos_profile.liveliness());
  EXPECT_EQ(
    actual_qos.liveliness_lease_duration(), subscription_qos_profile.liveliness_lease_duration());
}

TEST_F(QosRclcppTestFixture, test_best_available_policies_services) {
  // Test no errors occur when creating a service with best available policies
  rclcpp::Node node("test_create_service_with_best_available_policies");
  node.create_service<test_msgs::srv::Empty>(
    "test_best_available_service",
    [](
      std::shared_ptr<rmw_request_id_t>,
      std::shared_ptr<test_msgs::srv::Empty::Request>,
      std::shared_ptr<test_msgs::srv::Empty::Response>) {},
    rclcpp::BestAvailableQoS());
}

TEST_F(QosRclcppTestFixture, test_best_available_policies_clients) {
  // Test no errors occur when creating a client with best available policies
  rclcpp::Node node("test_create_client_with_best_available_policies");
  node.create_client<test_msgs::srv::Empty>(
    "test_best_available_client", rclcpp::BestAvailableQoS());
}
