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

#ifndef TEST_ROSIDL_BUFFER__TEST_BUFFER_IMPL_HPP_
#define TEST_ROSIDL_BUFFER__TEST_BUFFER_IMPL_HPP_

#include <atomic>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rosidl_buffer/buffer_impl_base.hpp"
#include "rosidl_buffer/cpu_buffer_impl.hpp"
#include "test_rosidl_buffer/msg/test_buffer_descriptor.hpp"

namespace test_rosidl_buffer
{

/// Process-wide counter of TestBufferImpl::to_cpu() invocations.
///
/// The launch tests use this to prove the test-to-test path never falls back
/// to CPU: if the backend's descriptor path is wired up correctly, the RMW
/// never needs to call `to_cpu()` on the publisher-side impl.
inline std::atomic<std::size_t> & to_cpu_call_count()
{
  static std::atomic<std::size_t> counter{0};
  return counter;
}

inline void reset_to_cpu_call_count()
{
  to_cpu_call_count().store(0, std::memory_order_relaxed);
}

/// Simple FNV-1a 64-bit hash for payload verification in descriptor round-trips.
inline std::uint64_t fnv1a_hash(const std::uint8_t * data, std::size_t size)
{
  constexpr std::uint64_t kOffsetBasis = 14695981039346656037ULL;
  constexpr std::uint64_t kPrime = 1099511628211ULL;
  std::uint64_t h = kOffsetBasis;
  for (std::size_t i = 0; i < size; ++i) {
    h ^= static_cast<std::uint64_t>(data[i]);
    h *= kPrime;
  }
  return h;
}

/// Minimal test-only implementation of rosidl::BufferImplBase.
///
/// Stores data in a std::vector<T>. Counts every call to `to_cpu()` in a
/// process-wide atomic so launch tests can assert the descriptor path is
/// exercised without any CPU fallback.
template<typename T>
class TestBufferImpl : public rosidl::BufferImplBase<T>
{
public:
  TestBufferImpl() = default;

  explicit TestBufferImpl(std::size_t n)
  : storage_(n) {}

  explicit TestBufferImpl(std::vector<T> data)
  : storage_(std::move(data)) {}

  ~TestBufferImpl() override = default;

  std::vector<T> & get_storage() {return storage_;}
  const std::vector<T> & get_storage() const {return storage_;}

  std::string get_backend_type() const override {return "test";}

  std::size_t size() const override {return storage_.size();}

  std::unique_ptr<rosidl::BufferImplBase<T>> to_cpu() const override
  {
    to_cpu_call_count().fetch_add(1, std::memory_order_relaxed);
    auto cpu = std::make_unique<rosidl::CpuBufferImpl<T>>();
    cpu->get_storage() = storage_;
    return cpu;
  }

  std::unique_ptr<rosidl::BufferImplBase<T>> clone() const override
  {
    return std::make_unique<TestBufferImpl<T>>(storage_);
  }

  /// Serialise this impl into a TestBufferDescriptor message.
  std::shared_ptr<msg::TestBufferDescriptor> create_descriptor() const
  {
    auto desc = std::make_shared<msg::TestBufferDescriptor>();
    desc->size = storage_.size();

    const std::size_t byte_count = storage_.size() * sizeof(T);
    std::vector<std::uint8_t> raw(byte_count);
    if (byte_count > 0) {
      std::memcpy(raw.data(), storage_.data(), byte_count);
    }
    desc->data_hash = fnv1a_hash(raw.data(), raw.size());
    desc->data = std::move(raw);
    return desc;
  }

  /// Deserialise a descriptor into a new TestBufferImpl.
  static std::unique_ptr<rosidl::BufferImplBase<T>> from_descriptor(
    const msg::TestBufferDescriptor & desc)
  {
    const std::size_t expected_bytes = desc.size * sizeof(T);
    if (desc.data.size() != expected_bytes) {
      throw std::runtime_error(
              "test_rosidl_buffer: descriptor payload size does not match element count");
    }
    const std::uint64_t recomputed = fnv1a_hash(desc.data.data(), desc.data.size());
    if (recomputed != desc.data_hash) {
      throw std::runtime_error(
              "test_rosidl_buffer: descriptor hash mismatch");
    }
    auto impl = std::make_unique<TestBufferImpl<T>>(desc.size);
    if (expected_bytes > 0) {
      std::memcpy(impl->storage_.data(), desc.data.data(), expected_bytes);
    }
    return impl;
  }

private:
  std::vector<T> storage_;
};

}  // namespace test_rosidl_buffer

#endif  // TEST_ROSIDL_BUFFER__TEST_BUFFER_IMPL_HPP_
