cmake_minimum_required(VERSION 3.5)

project(test_cli)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  # Default to C++14
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(rclcpp REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_executable(initial_params_rclcpp
    test/initial_params.cpp)
  ament_target_dependencies(initial_params_rclcpp
    "rclcpp")

  ament_add_pytest_test(test_params_yaml
    test/test_params_yaml.py
    ENV
      INITIAL_PARAMS_RCLCPP=$<TARGET_FILE:initial_params_rclcpp>
    TIMEOUT 120)
  set_tests_properties(test_params_yaml
    PROPERTIES DEPENDS
      initial_params_rclcpp)
endif()

ament_auto_package()
