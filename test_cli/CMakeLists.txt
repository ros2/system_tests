cmake_minimum_required(VERSION 3.20)

project(test_cli)

find_package(ament_cmake_auto REQUIRED)

if(BUILD_TESTING)
  if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
  endif()

  find_package(ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(ament_cmake_ros_core REQUIRED)
  find_package(test_msgs REQUIRED)

  find_package(Python3 REQUIRED COMPONENTS Interpreter)

  ament_lint_auto_find_test_dependencies()

  add_executable(initial_params_rclcpp
    test/initial_params.cpp)
  target_link_libraries(initial_params_rclcpp
    rclcpp::rclcpp
    ament_cmake_ros_core::ament_ros_defaults
  )

  ament_add_pytest_test(test_params_yaml
    test/test_params_yaml.py
    ENV
      INITIAL_PARAMS_RCLCPP=$<TARGET_FILE:initial_params_rclcpp>
      INITIAL_PARAMS_RCLPY=${CMAKE_CURRENT_LIST_DIR}/test/initial_params.py
    # https://github.com/ros2/system_tests/issues/365
    SKIP_TEST
  )
  if(TEST test_params_yaml)
    set_tests_properties(test_params_yaml
      PROPERTIES DEPENDS
        initial_params_rclcpp)
  endif()

  add_executable(name_maker_rclcpp
    test/name_maker.cpp)
  target_link_libraries(name_maker_rclcpp
    rclcpp::rclcpp
    test_msgs::test_msgs
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
