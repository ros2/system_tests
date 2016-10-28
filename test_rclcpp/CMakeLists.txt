cmake_minimum_required(VERSION 3.5)

project(test_rclcpp)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
else()
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
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

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations)
  foreach(rmw_implementation ${rmw_implementations})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  macro(custom_gtest target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

    ament_add_gtest(${target}${target_suffix} ${ARGN}
      ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation})
    if(TARGET ${target}${target_suffix})
      target_compile_definitions(${target}${target_suffix}
        PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
      add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
      foreach(typesupport_impl ${typesupport_impls})
        rosidl_target_interfaces(${target}${target_suffix}
          ${PROJECT_NAME} ${typesupport_impl})
      endforeach()
      ament_target_dependencies(${target}${target_suffix}
        "rclcpp${target_suffix}")
      target_include_directories(${target}${target_suffix} PUBLIC include)
    endif()
  endmacro()

  function(custom_executable target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

    add_executable(${target}${target_suffix} ${ARGN})
    target_compile_definitions(${target}${target_suffix}
      PUBLIC "RMW_IMPLEMENTATION=${rmw_implementation}")
    add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
    foreach(typesupport_impl ${typesupport_impls})
      rosidl_target_interfaces(${target}${target_suffix}
        ${PROJECT_NAME} ${typesupport_impl})
    endforeach()
    ament_target_dependencies(${target}${target_suffix}
      "rclcpp${target_suffix}")
  endfunction()

  macro(custom_launch_test test_name executable1 executable2)
    set(TEST_NAME "${test_name}")
    set(TEST_EXECUTABLE1 "$<TARGET_FILE:${executable1}${target_suffix}>")
    set(TEST_EXECUTABLE1_NAME "${executable1}")
    set(TEST_EXECUTABLE2 "$<TARGET_FILE:${executable2}${target_suffix}>")
    set(TEST_EXECUTABLE2_NAME "${executable2}")
    configure_file(
      test/test_two_executables.py.in
      ${test_name}${target_suffix}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${test_name}${target_suffix}_$<CONFIGURATION>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.configure"
    )
    ament_add_nose_test(${test_name}${target_suffix}
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIGURATION>.py"
      ${ARGN}
      ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
    )
    set_tests_properties(${test_name}${target_suffix}
      PROPERTIES DEPENDS "${executable1}${target_suffix} ${executable2}${target_suffix}"
    )
  endmacro()

  macro(targets)
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

    custom_gtest(gtest_publisher
      "test/test_publisher.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_client_wait_for_service_shutdown
      "test/test_client_wait_for_service_shutdown.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_executor
      "test/test_executor.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_repeated_publisher_subscriber
      "test/test_repeated_publisher_subscriber.cpp"
      TIMEOUT 15)
    custom_gtest(gtest_spin
      "test/test_spin.cpp"
      TIMEOUT 30)
    custom_gtest(gtest_subscription
      "test/test_subscription.cpp"
      TIMEOUT 60)
    custom_gtest(gtest_multiple_service_calls
      "test/test_multiple_service_calls.cpp"
      TIMEOUT 30)
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
      TIMEOUT 70)
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
    custom_launch_test(test_services_cpp
      test_services_server_cpp test_services_client_cpp
      TIMEOUT 60)

    custom_executable(test_client_scope_server_cpp "test/test_client_scope_server.cpp")
    custom_executable(test_client_scope_client_cpp "test/test_client_scope_client.cpp")
    target_include_directories(test_client_scope_client_cpp${target_suffix}
      PUBLIC ${GTEST_INCLUDE_DIRS})
    target_link_libraries(test_client_scope_client_cpp${target_suffix}
      ${GTEST_LIBRARIES})
    custom_launch_test(test_client_scope_cpp
      test_client_scope_server_cpp test_client_scope_client_cpp
      TIMEOUT 30)

    custom_executable(test_client_scope_consistency_server_cpp "test/test_client_scope_consistency_server.cpp")
    custom_executable(test_client_scope_consistency_client_cpp "test/test_client_scope_consistency_client.cpp")
    target_include_directories(test_client_scope_consistency_client_cpp${target_suffix}
      PUBLIC ${GTEST_INCLUDE_DIRS})
    target_link_libraries(test_client_scope_consistency_client_cpp${target_suffix}
      ${GTEST_LIBRARIES})
    custom_launch_test(test_client_scope_consistency_cpp
      test_client_scope_consistency_server_cpp test_client_scope_consistency_client_cpp
      TIMEOUT 30)
  endmacro()

  call_for_each_rmw_implementation(targets)
endif()  # BUILD_TESTING

# TODO should not install anything
ament_package()
