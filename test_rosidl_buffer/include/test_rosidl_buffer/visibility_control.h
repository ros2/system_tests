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

#ifndef TEST_ROSIDL_BUFFER__VISIBILITY_CONTROL_H_
#define TEST_ROSIDL_BUFFER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define TEST_ROSIDL_BUFFER_EXPORT __attribute__ ((dllexport))
    #define TEST_ROSIDL_BUFFER_IMPORT __attribute__ ((dllimport))
  #else
    #define TEST_ROSIDL_BUFFER_EXPORT __declspec(dllexport)
    #define TEST_ROSIDL_BUFFER_IMPORT __declspec(dllimport)
  #endif
  #ifdef TEST_ROSIDL_BUFFER_BUILDING_DLL
    #define TEST_ROSIDL_BUFFER_PUBLIC TEST_ROSIDL_BUFFER_EXPORT
  #else
    #define TEST_ROSIDL_BUFFER_PUBLIC TEST_ROSIDL_BUFFER_IMPORT
  #endif
  #define TEST_ROSIDL_BUFFER_LOCAL
#else
  #if __GNUC__ >= 4
    #define TEST_ROSIDL_BUFFER_PUBLIC __attribute__ ((visibility("default")))
    #define TEST_ROSIDL_BUFFER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define TEST_ROSIDL_BUFFER_PUBLIC
    #define TEST_ROSIDL_BUFFER_LOCAL
  #endif
#endif

#endif  // TEST_ROSIDL_BUFFER__VISIBILITY_CONTROL_H_
