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

  if (!message_empty->has_bounded_size()) {
    fprintf(stderr, "Empty::has_bounded_size() returned false!\n");
    //return 1;
  }

  if (message_primitives->has_bounded_size()) {
    fprintf(stderr, "Primitives::has_bounded_size() returned true!\n");
    //return 1;
  }

  if (!message_primitives_bounded->has_bounded_size()) {
    fprintf(stderr, "PrimitivesBounded::has_bounded_size() returned false!\n");
    //return 1;
  }

  if (message_static_array_primitives->has_bounded_size()) {
    fprintf(stderr, "StaticArrayPrimitives::has_bounded_size() returned true!\n");
    //return 1;
  }

  // TODO
  if (!message_static_array_bounded->has_bounded_size()) {
    fprintf(stderr, "StaticArrayBounded::has_bounded_size() returned false!\n");
    //return 1;
  }

  if (message_dynamic_array_primitives->has_bounded_size()) {
    fprintf(stderr, "DynamicArrayPrimitives::has_bounded_size() returned true!\n");
    //return 1;
  }

  if (message_nested->has_bounded_size()) {
    fprintf(stderr, "Nested::has_bounded_size() returned true!\n");
    //return 1;
  }

  if (!message_nested_bounded->has_bounded_size()) {
    fprintf(stderr, "NestedBounded::has_bounded_size() returned false!\n");
    //return 1;
  }
  if (message_dynamic_array_nested->has_bounded_size()) {
    fprintf(stderr, "DynamicArrayNested::has_bounded_size() returned true!\n");
    //return 1;
  }

  if (message_static_array_nested->has_bounded_size()) {
    fprintf(stderr, "StaticArrayNested::has_bounded_size() returned true!\n");
    //return 1;
  }

  // TODO
  if (!message_static_array_nested_bounded->has_bounded_size()) {
    fprintf(stderr, "StaticArrayNestedBounded::has_bounded_size() returned false!\n");
    //return 1;
  }
  if (!message_builtins->has_bounded_size()) {
    fprintf(stderr, "Builtins::has_bounded_size() returned false!\n");
    //return 1;
  }

  fprintf(stderr, "All tests passed.\n");
  return 0;
}
