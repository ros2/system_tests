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


#include <rclcpp/rclcpp.hpp>

#include "message_fixtures.hpp"

#include <test_communication/msg/primitives_bounded.hpp>
#include <test_communication/msg/static_array_bounded.hpp>
#include <test_communication/msg/static_array_nested_bounded.hpp>
#include <test_communication/msg/nested_bounded.hpp>

int main(int argc, char ** argv)
{
  auto message_empty = get_messages_empty()[0];
  auto message_primitives = get_messages_primitives()[0];
  auto message_static_array_primitives = get_messages_static_array_primitives()[0];
  auto message_dynamic_array_primitives = get_messages_dynamic_array_primitives()[0];
  auto message_nested = get_messages_nested()[0];
  auto message_dynamic_array_nested = get_messages_dynamic_array_nested()[0];
  auto message_static_array_nested = get_messages_static_array_nested()[0];
  auto message_builtins = get_messages_builtins()[0];

  test_communication::msg::PrimitivesBounded::SharedPtr message_primitives_bounded;
  test_communication::msg::StaticArrayBounded::SharedPtr message_static_array_bounded;
  test_communication::msg::StaticArrayNestedBounded::SharedPtr message_static_array_nested_bounded;
  test_communication::msg::NestedBounded::SharedPtr message_nested_bounded;

  // Check the types of all instantiated messages

  if (message_empty->is_dynamic()) {
    fprintf(stderr, "Empty::is_dynamic() returned true!\n");
    return 1;
  }

  if (!message_primitives->is_dynamic()) {
    fprintf(stderr, "Primitives::is_dynamic() returned false!\n");
    return 1;
  }

  if (message_primitives_bounded->is_dynamic()) {
    fprintf(stderr, "PrimitivesBounded::is_dynamic() returned true!\n");
    return 1;
  }

  if (!message_static_array_primitives->is_dynamic()) {
    fprintf(stderr, "StaticArrayPrimitives::is_dynamic() returned false!\n");
    return 1;
  }

  if (message_static_array_bounded->is_dynamic()) {
    fprintf(stderr, "StaticArrayBounded::is_dynamic() returned true!\n");
    return 1;
  }

  if (!message_dynamic_array_primitives->is_dynamic()) {
    fprintf(stderr, "DynamicArrayPrimitives::is_dynamic() returned false!\n");
    return 1;
  }

  if (!message_nested->is_dynamic()) {
    fprintf(stderr, "Nested::is_dynamic() returned false!\n");
    return 1;
  }

  if (message_nested_bounded->is_dynamic()) {
    fprintf(stderr, "NestedBounded::is_dynamic() returned true!\n");
    return 1;
  }
  if (!message_dynamic_array_nested->is_dynamic()) {
    fprintf(stderr, "DynamicArrayNested::is_dynamic() returned false!\n");
    return 1;
  }
  if (!message_static_array_nested->is_dynamic()) {
    fprintf(stderr, "StaticArrayNested::is_dynamic() returned false!\n");
    return 1;
  }
  if (message_static_array_nested_bounded->is_dynamic()) {
    fprintf(stderr, "StaticArrayNestedBounded::is_dynamic() returned true!\n");
    return 1;
  }
  if (message_builtins->is_dynamic()) {
    fprintf(stderr, "Builtins::is_dynamic() returned true!\n");
    return 1;
  }

  fprintf(stderr, "All tests passed.\n");
  return 0;
}
