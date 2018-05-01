// Copyright 2018 Open Source Robotics Foundation, Inc.
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

/*
 * This is a Fast-RTPS/Fast-CDR specific regression test for Ubuntu Bionic.
 * We encountered a segfault in some of the deserialization routines, and this
 * tests the same conditions that trigger that segfault
 *
 * Issue: https://github.com/ros2/rclcpp/issues/464
 */

#include <string>
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"
#include "rmw_fastrtps_cpp/ServiceTypeSupport.hpp"

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/service_type_support_decl.hpp"

#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters__rosidl_typesupport_introspection_cpp.hpp"

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"

using RequestTypeSupport_cpp =
  rmw_fastrtps_cpp::RequestTypeSupport<
  rosidl_typesupport_introspection_cpp::ServiceMembers,
  rosidl_typesupport_introspection_cpp::MessageMembers
  >;

using ResponseTypeSupport_cpp =
  rmw_fastrtps_cpp::ResponseTypeSupport<
  rosidl_typesupport_introspection_cpp::ServiceMembers,
  rosidl_typesupport_introspection_cpp::MessageMembers
  >;


class FastCdrTest : public ::testing::Test
{
protected:
  using SetParameters = rcl_interfaces::srv::SetParameters;

  void add_result(bool successful, std::string reason)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = successful;
    result.reason = reason;
    response_in->results.push_back(result);
  }

  void check_results()
  {
    for (size_t ii = 0; ii < 3; ++ii) {
      EXPECT_EQ(response_in->results[ii].successful, response_out->results[ii].successful);
      EXPECT_EQ(response_in->results[ii].reason, response_out->results[ii].reason);
    }
  }

  virtual void SetUp()
  {
    response_in = std::make_shared<SetParameters::Response>();
    response_out = std::make_shared<SetParameters::Response>();

    add_result(true, "foo");
    add_result(false, "bar");
    add_result(true, "");

    auto handle =
      rosidl_typesupport_introspection_cpp::get_service_type_support_handle<SetParameters>();
    auto service_members = const_cast<rosidl_typesupport_introspection_cpp::ServiceMembers *>(
      static_cast<const ::rosidl_typesupport_introspection_cpp::ServiceMembers *>(
        handle->data));
    response_ts = std::make_shared<ResponseTypeSupport_cpp>(service_members);
  }

  std::shared_ptr<SetParameters::Response> response_in, response_out;
  std::shared_ptr<ResponseTypeSupport_cpp> response_ts;
};

TEST_F(FastCdrTest, roundtrip_prealloc) {
  eprosima::fastcdr::FastBuffer buffer;
  eprosima::fastcdr::Cdr ser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);

  response_ts->serializeROSmessage(response_in.get(), ser);

  response_out->results.resize(3);
  ASSERT_NO_THROW(response_ts->deserializeROSmessage(&buffer, response_out.get()));
  check_results();
}

TEST_F(FastCdrTest, roundtrip_no_prealloc) {
  eprosima::fastcdr::FastBuffer buffer;
  eprosima::fastcdr::Cdr ser(buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN,
    eprosima::fastcdr::Cdr::DDS_CDR);

  response_ts->serializeROSmessage(response_in.get(), ser);
  ASSERT_NO_THROW(response_ts->deserializeROSmessage(&buffer, response_out.get()));
  check_results();
}

int main(int argc, char ** argv)
{
  // NOTE: use custom main to ensure that rclcpp::init is called only once
  rclcpp::init(0, nullptr);
  ::testing::InitGoogleTest(&argc, argv);
  int ret = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return ret;
}
