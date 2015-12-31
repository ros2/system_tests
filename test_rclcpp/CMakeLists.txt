cmake_minimum_required(VERSION 2.8.3)

project(test_rclcpp)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
else()
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

find_package(ament_cmake REQUIRED)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/UInt32.msg"
  )

  set(service_files
    "srv/AddTwoInts.srv"
  )

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
    ${service_files}
    SKIP_INSTALL
  )

  if(NOT WIN32)
    set(test_executable_extension "")
    set(test_executable_subfolder "")
  else()
    set(test_executable_extension ".exe")
    set(test_executable_subfolder "/${CMAKE_BUILD_TYPE}")
    if("${test_executable_subfolder}" STREQUAL "/")
      set(test_executable_subfolder "/Debug")
    endif()
  endif()

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  macro(custom_gtest target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impl "${rmw_implementation}")

    ament_add_gtest(${target}${target_suffix} ${ARGN})
    if(TARGET ${target}${target_suffix})
      target_link_libraries(${target}${target_suffix}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(${target}${target_suffix}
        PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
      add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
      rosidl_target_interfaces(${target}${target_suffix}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(${target}${target_suffix}
        "rclcpp${target_suffix}")
    endif()
  endmacro()

  function(custom_executable target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impl "${rmw_implementation}")

    add_executable(${target}${target_suffix} ${ARGN})
    target_compile_definitions(${target}${target_suffix}
      PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
    rosidl_target_interfaces(${target}${target_suffix}
      ${PROJECT_NAME} ${typesupport_impl})
    ament_target_dependencies(${target}${target_suffix}
      "rclcpp${target_suffix}")
  endfunction()

  macro(custom_launch_test test_name executable1 executable2)
    set(TEST_NAME "${test_name}")
    set(TEST_EXECUTABLE1 "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/${executable1}${target_suffix}${test_executable_extension}")
    set(TEST_EXECUTABLE1_NAME "${executable1}")
    set(TEST_EXECUTABLE2 "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/${executable2}${target_suffix}${test_executable_extension}")
    set(TEST_EXECUTABLE2_NAME "${executable2}")
    configure_file(
      test/test_two_executables.py.in
      ${test_name}${target_suffix}.py
      @ONLY
    )
    ament_add_nose_test(${test_name}${target_suffix} "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py" ${ARGN})
    set_tests_properties(${test_name}${target_suffix} PROPERTIES DEPENDS "${executable1}${target_suffix} ${executable2}${target_suffix}")
  endmacro()

  macro(targets)
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

    custom_gtest(gtest_publisher
      "test/test_publisher.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_executor
      "test/test_executor.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_repeated_publisher_subscriber
      "test/test_repeated_publisher_subscriber.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_spin
      "test/test_spin.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_subscription
      "test/test_subscription.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_multiple_service_calls
      "test/test_multiple_service_calls.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_timer
      "test/test_timer.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_timeout_subscriber
      "test/test_timeout_subscriber.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_intra_process
      "test/test_intra_process.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_multithreaded
      "test/test_multithreaded.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_local_parameters
      "test/test_local_parameters.cpp"
      TIMEOUT 30)

    # Parameter tests single implementation
    custom_executable(test_parameters_server_cpp "test/test_parameters_server.cpp")
    custom_executable(test_remote_parameters_cpp "test/test_remote_parameters.cpp")
    target_include_directories(test_remote_parameters_cpp${target_suffix}
      PUBLIC ${GTEST_INCLUDE_DIRS})
    target_link_libraries(test_remote_parameters_cpp${target_suffix}
      ${GTEST_LIBRARIES})
    custom_launch_test(test_parameter_server_cpp
      test_parameters_server_cpp test_remote_parameters_cpp
      TIMEOUT 15)

    # Service tests single implementation
    custom_executable(test_services_server_cpp "test/test_services_server.cpp")
    custom_executable(test_services_client_cpp "test/test_services_client.cpp")
    target_include_directories(test_services_client_cpp${target_suffix}
      PUBLIC ${GTEST_INCLUDE_DIRS})
    target_link_libraries(test_services_client_cpp${target_suffix}
      ${GTEST_LIBRARIES})
    # https://github.com/eProsima/ROS-RMW-Fast-RTPS-cpp/issues/11
    if(NOT ${rmw_implementation} STREQUAL "rmw_fastrtps_cpp")
      custom_launch_test(test_services_cpp
        test_services_server_cpp test_services_client_cpp
        TIMEOUT 30)
    endif()
  endmacro()

  call_for_each_rmw_implementation(targets)
endif()  # AMENT_ENABLE_TESTING

# TODO should not install anything
ament_package()
