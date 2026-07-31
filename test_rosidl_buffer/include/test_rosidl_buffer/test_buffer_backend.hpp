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

#ifndef TEST_ROSIDL_BUFFER__TEST_BUFFER_BACKEND_HPP_
#define TEST_ROSIDL_BUFFER__TEST_BUFFER_BACKEND_HPP_

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rmw/types.h"
#include "rmw/topic_endpoint_info.h"
#include "rosidl_buffer_backend/buffer_backend.hpp"

#include "test_rosidl_buffer/visibility_control.h"

namespace test_rosidl_buffer
{

/// Minimal pluginlib-exported BufferBackend used by system tests.
///
/// Reports its backend type as "test" and only returns a descriptor when the
/// discovered peer also advertises the "test" backend. When the peer does not,
/// create_descriptor_with_endpoint() returns nullptr so the serialization
/// layer falls back to CPU — this is exactly the test-to-cpu scenario.
class TEST_ROSIDL_BUFFER_PUBLIC TestBufferBackend : public rosidl::BufferBackend
{
public:
  TestBufferBackend();
  ~TestBufferBackend() override = default;

  std::string get_backend_type() const override {return "test";}

  std::string get_backend_metadata() const override {return "version=test";}

  const rosidl_message_type_support_t * get_descriptor_type_support() const override;

  std::shared_ptr<void> create_empty_descriptor() const override;

  std::shared_ptr<void> create_descriptor_with_endpoint(
    const void * impl,
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  std::unique_ptr<void, void (*)(void *)> from_descriptor_with_endpoint(
    const void * descriptor,
    const rmw_topic_endpoint_info_t & endpoint_info) const override;

  std::pair<bool, std::vector<std::set<std::uint32_t>>> on_discovering_endpoint(
    const rmw_topic_endpoint_info_t & endpoint_info,
    const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
    const std::unordered_map<std::string, std::string> & endpoint_supported_backends) override;

private:
  static std::size_t gid_hash(const std::uint8_t * gid)
  {
    std::size_t h = 0;
    for (std::size_t i = 0; i < RMW_GID_STORAGE_SIZE; ++i) {
      h ^= std::hash<std::uint8_t>{}(gid[i]) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
  }

  mutable std::mutex compat_mutex_;
  std::unordered_map<std::size_t, bool> endpoint_compat_cache_;
};

}  // namespace test_rosidl_buffer

#endif  // TEST_ROSIDL_BUFFER__TEST_BUFFER_BACKEND_HPP_
