cmake_minimum_required(VERSION 3.5)

project(test_cli_remapping)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(BUILD_TESTING)
  find_package(ament_cmake REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  add_executable(name_maker_rclcpp
    test/name_maker.cpp)
  target_compile_features(name_maker_rclcpp PRIVATE cxx_lambdas)
  ament_target_dependencies(name_maker_rclcpp
    "rclcpp"
    "test_msgs")

  ament_add_pytest_test(test_cli_remapping
    test/test_cli_remapping.py
    ENV
      NAME_MAKER_RCLCPP=$<TARGET_FILE:name_maker_rclcpp>
      NAME_MAKER_RCLPY=${CMAKE_CURRENT_SOURCE_DIR}/test/name_maker.py
    TIMEOUT 120)
  set_tests_properties(test_cli_remapping
    PROPERTIES DEPENDS
      name_maker_rclcpp)
endif()

ament_auto_package()
