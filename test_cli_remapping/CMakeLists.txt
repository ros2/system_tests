cmake_minimum_required(VERSION 3.5)

project(test_cli_remapping)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  # Default to C++17
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
  endif()
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(test_msgs REQUIRED)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_lint_auto_find_test_dependencies()

  add_executable(name_maker_rclcpp
    test/name_maker.cpp)
  target_link_libraries(name_maker_rclcpp
    rclcpp::rclcpp
    ${test_msgs_TARGETS}
  )

  add_launch_test(
    test/test_cli_remapping.py
    TARGET test_cli_remapping
    ENV
      NAME_MAKER_RCLCPP=$<TARGET_FILE:name_maker_rclcpp>
      NAME_MAKER_RCLPY=${CMAKE_CURRENT_SOURCE_DIR}/test/name_maker.py
    TIMEOUT 120)
  set_tests_properties(test_cli_remapping
    PROPERTIES DEPENDS
      name_maker_rclcpp)
endif()

ament_auto_package()
