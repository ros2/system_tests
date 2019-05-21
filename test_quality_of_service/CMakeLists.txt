cmake_minimum_required(VERSION 3.5)

project(test_quality_of_service)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcutils)
  find_package(rmw REQUIRED)
  find_package(std_msgs REQUIRED)

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations)
  foreach(rmw_implementation ${rmw_implementations})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  include_directories(include)
  ament_lint_auto_find_test_dependencies()

  # build a gtest executable with an implemented main
  function(custom_gtest_executable target)
    ament_add_gtest_executable(${target} ${ARGN} test/${target}.cpp
      test/qos_test_node.cpp
      test/qos_test_publisher.cpp
      test/qos_test_subscriber.cpp
      test/qos_utilities.cpp)
    target_compile_definitions(${target}  PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    ament_target_dependencies(${target} rclcpp rcutils std_msgs)
    install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
  endfunction()

  custom_gtest_executable(test_deadline "test/test_deadline.cpp")
  custom_gtest_executable(test_lifespan "test/test_lifespan.cpp")
  custom_gtest_executable(test_liveliness "test/test_liveliness.cpp")

endif()

ament_package()
