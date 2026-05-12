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

#include "test_rosidl_buffer/test_buffer_backend.hpp"

#include <cstring>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "rosidl_typesupport_cpp/message_type_support.hpp"

#include "test_rosidl_buffer/msg/test_buffer_descriptor.hpp"
#include "test_rosidl_buffer/test_buffer_impl.hpp"

namespace test_rosidl_buffer
{

TestBufferBackend::TestBufferBackend() = default;

const rosidl_message_type_support_t * TestBufferBackend::get_descriptor_type_support() const
{
  return rosidl_typesupport_cpp::get_message_type_support_handle<msg::TestBufferDescriptor>();
}

std::shared_ptr<void> TestBufferBackend::create_empty_descriptor() const
{
  return std::make_shared<msg::TestBufferDescriptor>();
}

std::pair<bool, std::vector<std::set<std::uint32_t>>> TestBufferBackend::on_discovering_endpoint(
  const rmw_topic_endpoint_info_t & endpoint_info,
  [[maybe_unused]] const std::vector<rmw_topic_endpoint_info_t> & existing_endpoints,
  const std::unordered_map<std::string, std::string> & endpoint_supported_backends)
{
  (void)existing_endpoints;

  const bool peer_supports_test =
    endpoint_supported_backends.find("test") != endpoint_supported_backends.end();

  {
    std::lock_guard<std::mutex> lock(compat_mutex_);
    endpoint_compat_cache_[gid_hash(endpoint_info.endpoint_gid)] = peer_supports_test;
  }

  return {peer_supports_test, {}};
}

std::shared_ptr<void> TestBufferBackend::create_descriptor_with_endpoint(
  const void * impl,
  const rmw_topic_endpoint_info_t & endpoint_info) const
{
  {
    std::lock_guard<std::mutex> lock(compat_mutex_);
    const auto it = endpoint_compat_cache_.find(gid_hash(endpoint_info.endpoint_gid));
    // If we already know the peer is not "test"-capable, signal the RMW to fall
    // back to CPU serialization. Unknown peers default to "try descriptor" so
    // early publishes that race discovery still get a chance on the test path.
    if (it != endpoint_compat_cache_.end() && !it->second) {
      return nullptr;
    }
  }

  const auto * test_impl = static_cast<const TestBufferImpl<std::uint8_t> *>(impl);
  return test_impl->create_descriptor();
}

std::unique_ptr<void, void (*)(void *)> TestBufferBackend::from_descriptor_with_endpoint(
  const void * descriptor,
  [[maybe_unused]] const rmw_topic_endpoint_info_t & endpoint_info) const
{
  (void)endpoint_info;
  const auto & desc = *static_cast<const msg::TestBufferDescriptor *>(descriptor);
  auto impl = TestBufferImpl<std::uint8_t>::from_descriptor(desc);
  return {
    impl.release(),
    [](void * p) {delete static_cast<rosidl::BufferImplBase<std::uint8_t> *>(p);}
  };
}

}  // namespace test_rosidl_buffer

PLUGINLIB_EXPORT_CLASS(
  test_rosidl_buffer::TestBufferBackend,
  rosidl::BufferBackend)
