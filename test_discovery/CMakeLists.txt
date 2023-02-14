cmake_minimum_required(VERSION 3.5)
project(test_discovery)

find_package(ament_cmake_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(test_msgs REQUIRED)

add_executable(publish_once src/publish_once.cpp)
target_compile_features(publish_once PUBLIC cxx_std_17)
target_link_libraries(publish_once PRIVATE
    rclcpp::rclcpp
    ${test_msgs_TARGETS}
)

add_executable(subscribe_once src/subscribe_once.cpp)
target_compile_features(subscribe_once PUBLIC cxx_std_17)
target_link_libraries(subscribe_once PRIVATE
    rclcpp::rclcpp
    ${test_msgs_TARGETS}
)

install(TARGETS publish_once subscribe_once
    DESTINATION lib/${PROJECT_NAME})

install(FILES
        roottests/test_discovery.py
        roottests/conftest.py
    DESTINATION share/${PROJECT_NAME}/roottests/)

install(PROGRAMS
    scripts/run_root_tests.py
    DESTINATION lib/${PROJECT_NAME}/)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()

    ament_add_pytest_test(test_discovery tests/test_discovery.py TIMEOUT 800)
endif()

ament_package()