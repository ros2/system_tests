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

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "rmw/topic_endpoint_info.h"
#include "rmw/types.h"

#include "rosidl_buffer/cpu_buffer_impl.hpp"
#include "rosidl_buffer_backend/buffer_backend.hpp"

#include "test_rosidl_buffer/msg/test_buffer_descriptor.hpp"
#include "test_rosidl_buffer/test_buffer_backend.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

using test_rosidl_buffer::TestBufferBackend;
using test_rosidl_buffer::TestBufferImpl;

namespace
{

rmw_topic_endpoint_info_t make_dummy_endpoint()
{
  rmw_topic_endpoint_info_t info;
  std::memset(&info, 0, sizeof(info));
  info.topic_type = "test_rosidl_buffer/msg/ByteArray";
  return info;
}

}  // namespace

TEST(TestBufferImplUnit, BackendTypeAndSize)
{
  TestBufferImpl<std::uint8_t> impl(std::vector<std::uint8_t>{1, 2, 3});
  EXPECT_EQ("test", impl.get_backend_type());
  EXPECT_EQ(3u, impl.size());
}

TEST(TestBufferImplUnit, ToCpuIncrementsCounter)
{
  test_rosidl_buffer::reset_to_cpu_call_count();
  TestBufferImpl<std::uint8_t> impl(std::vector<std::uint8_t>{1, 2, 3});

  ASSERT_EQ(0u, test_rosidl_buffer::to_cpu_call_count().load());
  auto cpu = impl.to_cpu();
  ASSERT_NE(nullptr, cpu);
  EXPECT_EQ(1u, test_rosidl_buffer::to_cpu_call_count().load());

  auto * cpu_impl = dynamic_cast<rosidl::CpuBufferImpl<std::uint8_t> *>(cpu.get());
  ASSERT_NE(nullptr, cpu_impl);
  EXPECT_EQ((std::vector<std::uint8_t>{1, 2, 3}), cpu_impl->get_storage());
}

TEST(TestBufferImplUnit, DescriptorRoundTrip)
{
  test_rosidl_buffer::reset_to_cpu_call_count();

  std::vector<std::uint8_t> bytes(256);
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    bytes[i] = static_cast<std::uint8_t>(i);
  }
  TestBufferImpl<std::uint8_t> original(bytes);

  auto desc = original.create_descriptor();
  ASSERT_NE(nullptr, desc);
  EXPECT_EQ(256u, desc->size);
  EXPECT_EQ(256u, desc->data.size());
  EXPECT_NE(0u, desc->data_hash);

  auto reconstructed = TestBufferImpl<std::uint8_t>::from_descriptor(*desc);
  ASSERT_NE(nullptr, reconstructed);
  auto * typed = dynamic_cast<TestBufferImpl<std::uint8_t> *>(reconstructed.get());
  ASSERT_NE(nullptr, typed);
  EXPECT_EQ(bytes, typed->get_storage());

  // Round-tripping must not require a to_cpu() conversion.
  EXPECT_EQ(0u, test_rosidl_buffer::to_cpu_call_count().load());
}

TEST(TestBufferImplUnit, DescriptorHashMismatchThrows)
{
  test_rosidl_buffer::msg::TestBufferDescriptor desc;
  desc.size = 3;
  desc.data = std::vector<std::uint8_t>{1, 2, 3};
  desc.data_hash = 0;   // Intentionally wrong.

  EXPECT_THROW(
    TestBufferImpl<std::uint8_t>::from_descriptor(desc),
    std::runtime_error);
}

TEST(TestBufferBackendUnit, BackendTypeAndMetadata)
{
  TestBufferBackend backend;
  EXPECT_EQ("test", backend.get_backend_type());
  EXPECT_EQ("version=test", backend.get_backend_metadata());
}

TEST(TestBufferBackendUnit, EmptyDescriptorAndTypeSupportConsistent)
{
  TestBufferBackend backend;
  auto empty = backend.create_empty_descriptor();
  ASSERT_NE(nullptr, empty);

  const auto * ts = backend.get_descriptor_type_support();
  ASSERT_NE(nullptr, ts);
  ASSERT_NE(nullptr, ts->data);
}

TEST(TestBufferBackendUnit, DescriptorRoundTripThroughBackend)
{
  TestBufferBackend backend;
  const auto endpoint = make_dummy_endpoint();

  const std::vector<std::uint8_t> bytes{10, 20, 30, 40, 50};
  TestBufferImpl<std::uint8_t> impl(bytes);

  // Pretend the peer supports us so the compat cache does not short-circuit.
  std::unordered_map<std::string, std::string> peer_backends;
  peer_backends["test"] = "version=test";
  (void)backend.on_discovering_endpoint(endpoint, {}, peer_backends);

  auto desc = backend.create_descriptor_with_endpoint(&impl, endpoint);
  ASSERT_NE(nullptr, desc);

  auto reconstructed = backend.from_descriptor_with_endpoint(desc.get(), endpoint);
  ASSERT_NE(nullptr, reconstructed.get());

  auto * base = static_cast<rosidl::BufferImplBase<std::uint8_t> *>(reconstructed.get());
  auto * typed = dynamic_cast<TestBufferImpl<std::uint8_t> *>(base);
  ASSERT_NE(nullptr, typed);
  EXPECT_EQ(bytes, typed->get_storage());
}

TEST(TestBufferBackendUnit, CreateDescriptorReturnsNullForIncompatiblePeer)
{
  TestBufferBackend backend;
  const auto endpoint = make_dummy_endpoint();

  // Peer does NOT advertise "test".
  std::unordered_map<std::string, std::string> peer_backends;
  peer_backends["other"] = "x";
  const auto disc = backend.on_discovering_endpoint(endpoint, {}, peer_backends);
  EXPECT_FALSE(disc.first);

  TestBufferImpl<std::uint8_t> impl(std::vector<std::uint8_t>{1, 2, 3});
  auto desc = backend.create_descriptor_with_endpoint(&impl, endpoint);
  EXPECT_EQ(nullptr, desc) <<
    "Backend must return nullptr for peers that do not advertise the 'test' backend, "
    "so the serialization layer can fall back to CPU.";
}

TEST(TestBufferBackendUnit, DescriptorSizeUnderLimit)
{
  // Largest payload we would realistically send through this test backend.
  // Pick something big but within rosidl::kMaxBufferDescriptorSize after the
  // small fixed overhead for size/hash fields.
  constexpr std::size_t kPayload = 3584;
  static_assert(
    kPayload + 128 < rosidl::kMaxBufferDescriptorSize,
    "payload + descriptor header overhead must fit within kMaxBufferDescriptorSize");

  TestBufferImpl<std::uint8_t> impl(std::vector<std::uint8_t>(kPayload, 0xAB));
  auto desc = impl.create_descriptor();
  ASSERT_NE(nullptr, desc);

  // Approximate serialized size: uint8[] len + header fixed overhead.
  const std::size_t approx_serialized_bytes =
    desc->data.size() + sizeof(desc->size) + sizeof(desc->data_hash) +
    /* typical CDR padding/length fields */ 32u;
  EXPECT_LE(approx_serialized_bytes, rosidl::kMaxBufferDescriptorSize);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
