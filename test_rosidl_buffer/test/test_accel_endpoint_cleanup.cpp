// Copyright 2026 Open Source Robotics Foundation, Inc.
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

// A buffer-aware publisher and a subscription using the `test` backend in the
// same process, destroyed in either order: check that the RMW releases
// everything it allocated for them once the context is shut down.
//
// Allocations are counted by replacing the global operator new/delete of this
// test binary, so that memory allocated inside the RMW and its middleware is
// seen too, without depending on a particular RMW implementation.

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rosidl_buffer/buffer.hpp"
#include "test_rosidl_buffer/msg/byte_array.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

namespace
{
std::atomic<int64_t> g_live_allocations{0};
}  // namespace

void * operator new(std::size_t size)
{
  void * ptr = std::malloc(size ? size : 1);
  if (!ptr) {
    throw std::bad_alloc();
  }
  ++g_live_allocations;
  return ptr;
}

void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  void * ptr = std::malloc(size ? size : 1);
  if (ptr) {
    ++g_live_allocations;
  }
  return ptr;
}

void * operator new[](std::size_t size)
{
  return operator new(size);
}

void * operator new[](std::size_t size, const std::nothrow_t & tag) noexcept
{
  return operator new(size, tag);
}

void operator delete(void * ptr) noexcept
{
  if (ptr) {
    --g_live_allocations;
    std::free(ptr);
  }
}

void operator delete(void * ptr, std::size_t) noexcept
{
  operator delete(ptr);
}

void operator delete(void * ptr, const std::nothrow_t &) noexcept
{
  operator delete(ptr);
}

void operator delete[](void * ptr) noexcept
{
  operator delete(ptr);
}

void operator delete[](void * ptr, std::size_t) noexcept
{
  operator delete(ptr);
}

void operator delete[](void * ptr, const std::nothrow_t &) noexcept
{
  operator delete(ptr);
}

namespace
{
using ByteArray = test_rosidl_buffer::msg::ByteArray;
using namespace std::chrono_literals;

constexpr int kCycles = 5;
// Leaving the publisher's or the subscription's middleware entities behind costs
// thousands of allocations per cycle.
constexpr int64_t kMaxAllocationsPerCycle = 0;

enum class DestroyOrder { SubscriptionFirst, PublisherFirst };

// Creates a context, a publisher and a subscription accepting any buffer backend,
// waits until a message is received through the `test` backend, destroys them
// in the given order and shuts the context down.
void run_cycle(DestroyOrder order)
{
  auto context = std::make_shared<rclcpp::Context>();
  context->init(0, nullptr);
  {
    rclcpp::NodeOptions node_options;
    node_options.context(context);
    auto node = std::make_shared<rclcpp::Node>("test_accel_endpoint_cleanup", node_options);

    std::string backend;
    rclcpp::SubscriptionOptions sub_options;
    sub_options.acceptable_buffer_backends = "any";
    auto sub = node->create_subscription<ByteArray>(
      "accel_endpoint_cleanup", 10,
      [&backend](const ByteArray & msg) {backend = msg.data.get_backend_type();}, sub_options);
    auto pub = node->create_publisher<ByteArray>("accel_endpoint_cleanup", 10);

    rclcpp::ExecutorOptions executor_options;
    executor_options.context = context;
    rclcpp::executors::SingleThreadedExecutor executor(executor_options);
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + 10s;
    while (backend != "test" && std::chrono::steady_clock::now() < deadline) {
      ByteArray msg;
      msg.data = rosidl::Buffer<std::uint8_t>(
        std::make_unique<test_rosidl_buffer::TestBufferImpl<std::uint8_t>>(
          std::vector<std::uint8_t>(64, 1)));
      pub->publish(msg);
      executor.spin_some(50ms);
    }
    executor.remove_node(node);
    EXPECT_EQ("test", backend) << "no message received through the test backend";

    if (order == DestroyOrder::SubscriptionFirst) {
      sub.reset();
      pub.reset();
    } else {
      pub.reset();
      sub.reset();
    }
  }
  context->shutdown("test cycle done");
}

void check_no_growth(DestroyOrder order)
{
  // The first cycle loads plugins and fills one-time caches.
  run_cycle(order);
  const int64_t baseline = g_live_allocations.load();
  for (int i = 0; i < kCycles; ++i) {
    run_cycle(order);
  }
  const int64_t growth = g_live_allocations.load() - baseline;
  EXPECT_LE(growth, kMaxAllocationsPerCycle * kCycles)
    << growth << " allocations still alive after " << kCycles << " cycles";
}
}  // namespace

TEST(TestAccelEndpointCleanup, publisher_destroyed_first) {
  check_no_growth(DestroyOrder::PublisherFirst);
}

TEST(TestAccelEndpointCleanup, subscription_destroyed_first) {
  check_no_growth(DestroyOrder::SubscriptionFirst);
}
