cmake_minimum_required(VERSION 3.5)

project(test_rclcpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(rclcpp REQUIRED)
  find_package(rcpputils REQUIRED)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(launch_testing_ament_cmake REQUIRED)

  find_package(ament_cmake_gtest REQUIRED)

  # finding gtest once in the highest scope
  # prevents finding it repeatedly in each local scope
  ament_find_gtest()

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

  rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  ament_add_gtest_executable(gtest_avoid_ros_namespace_conventions_qos
    test/test_avoid_ros_namespace_conventions_qos.cpp
  )
  target_include_directories(gtest_avoid_ros_namespace_conventions_qos PRIVATE include)
  target_link_libraries(gtest_avoid_ros_namespace_conventions_qos "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_publisher test/test_publisher.cpp)
  target_include_directories(gtest_publisher PRIVATE include)
  target_link_libraries(gtest_publisher "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_client_wait_for_service_shutdown
    test/test_client_wait_for_service_shutdown.cpp
  )
  target_include_directories(gtest_client_wait_for_service_shutdown PRIVATE include)
  target_link_libraries(gtest_client_wait_for_service_shutdown "${cpp_typesupport_target}" rclcpp::rclcpp rcpputils::rcpputils)

  ament_add_gtest_executable(gtest_executor test/test_executor.cpp)
  target_include_directories(gtest_executor PRIVATE include)
  target_link_libraries(gtest_executor "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_repeated_publisher_subscriber test/test_repeated_publisher_subscriber.cpp)
  target_include_directories(gtest_repeated_publisher_subscriber PRIVATE include)
  target_link_libraries(gtest_repeated_publisher_subscriber "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_spin test/test_spin.cpp)
  target_include_directories(gtest_spin PRIVATE include)
  target_link_libraries(gtest_spin "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_subscription test/test_subscription.cpp)
  target_include_directories(gtest_subscription PRIVATE include)
  target_link_libraries(gtest_subscription "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_multiple_service_calls test/test_multiple_service_calls.cpp)
  target_include_directories(gtest_multiple_service_calls PRIVATE include)
  target_link_libraries(gtest_multiple_service_calls "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_timer test/test_timer.cpp)
  target_include_directories(gtest_timer PRIVATE include)
  target_link_libraries(gtest_timer rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_timeout_subscriber test/test_timeout_subscriber.cpp)
  target_include_directories(gtest_timeout_subscriber PRIVATE include)
  target_link_libraries(gtest_timeout_subscriber "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_intra_process test/test_intra_process.cpp)
  target_include_directories(gtest_intra_process PRIVATE include)
  target_link_libraries(gtest_intra_process "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_multithreaded test/test_multithreaded.cpp)
  target_include_directories(gtest_multithreaded PRIVATE include)
  target_link_libraries(gtest_multithreaded "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_local_parameters test/test_local_parameters.cpp)
  target_include_directories(gtest_local_parameters PRIVATE include)
  target_link_libraries(gtest_local_parameters rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_services_in_constructor test/test_services_in_constructor.cpp)
  target_include_directories(gtest_services_in_constructor PRIVATE include)
  target_link_libraries(gtest_services_in_constructor "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(gtest_waitable test/test_waitable.cpp)
  target_include_directories(gtest_waitable PRIVATE include)
  target_link_libraries(gtest_waitable rclcpp::rclcpp)

  # Test node names
  add_executable(node_with_name test/node_with_name.cpp)
  target_link_libraries(node_with_name rclcpp::rclcpp)
  add_executable(node_name_list test/node_name_list.cpp)
  target_link_libraries(node_name_list rclcpp::rclcpp)
  add_executable(node_check_names test/node_check_names.cpp)
  target_link_libraries(node_check_names rclcpp::rclcpp)

  add_executable(test_sigint_handler test/test_sigint_handler.cpp)
  target_link_libraries(test_sigint_handler rclcpp::rclcpp)

  add_executable(test_sigterm_handler test/test_sigterm_handler.cpp)
  target_link_libraries(test_sigterm_handler rclcpp::rclcpp)

  # Parameter tests single implementation
  add_executable(test_parameters_server_cpp test/test_parameters_server.cpp)
  target_link_libraries(test_parameters_server_cpp rclcpp::rclcpp)

  ament_add_gtest_executable(test_remote_parameters_cpp test/test_remote_parameters.cpp)
  target_link_libraries(test_remote_parameters_cpp rclcpp::rclcpp)

  # Service tests single implementation
  add_executable(test_services_server_cpp test/test_services_server.cpp)
  target_link_libraries(test_services_server_cpp "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(test_services_client_cpp test/test_services_client.cpp)
  target_link_libraries(test_services_client_cpp "${cpp_typesupport_target}" rclcpp::rclcpp)

  add_executable(test_client_scope_server_cpp test/test_client_scope_server.cpp)
  target_link_libraries(test_client_scope_server_cpp "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(test_client_scope_client_cpp test/test_client_scope_client.cpp)
  target_link_libraries(test_client_scope_client_cpp "${cpp_typesupport_target}" rclcpp::rclcpp)

  add_executable(test_client_scope_consistency_server_cpp test/test_client_scope_consistency_server.cpp)
  target_link_libraries(test_client_scope_consistency_server_cpp "${cpp_typesupport_target}" rclcpp::rclcpp)

  ament_add_gtest_executable(test_client_scope_consistency_client_cpp test/test_client_scope_consistency_client.cpp)
  target_link_libraries(test_client_scope_consistency_client_cpp "${cpp_typesupport_target}" rclcpp::rclcpp)

  macro(custom_launch_test_two_executables test_name executable1 executable2)
    cmake_parse_arguments(_ARG "" "ARGS1;ARGS2;RMW1;RMW2" "" ${ARGN})
    set(TEST_NAME "${test_name}")
    set(TEST_EXECUTABLE1 "$<TARGET_FILE:${executable1}>")
    set(TEST_EXECUTABLE1_ARGS "${_ARG_ARGS1}")
    set(TEST_EXECUTABLE1_NAME "${executable1}")
    set(TEST_RMW_IMPLEMENTATION1 "${_ARG_RMW1}")
    set(TEST_EXECUTABLE2 "$<TARGET_FILE:${executable2}>")
    set(TEST_EXECUTABLE2_ARGS "${_ARG_ARGS2}")
    set(TEST_EXECUTABLE2_NAME "${executable2}")
    set(TEST_RMW_IMPLEMENTATION2 "${_ARG_RMW2}")
    configure_file(
      test/test_two_executables.py.in
      ${test_name}${target_suffix}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET "${test_name}${target_suffix}"
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ${_ARG_UNPARSED_ARGUMENTS}
    )
    if(TEST ${test_name}${target_suffix})
      set_tests_properties(${test_name}${target_suffix}
        PROPERTIES DEPENDS "${executable1}${target_suffix} ${executable2}${target_suffix}"
      )
    endif()
  endmacro()

  macro(custom_launch_n_nodes num_nodes)
    set(TEST_EXECUTABLE1 "$<TARGET_FILE:node_with_name>")
    set(TEST_EXECUTABLE2 "$<TARGET_FILE:node_check_names>")
    set(TEST_RMW_IMPLEMENTATION "${rmw_implementation}")
    set(TEST_NUM_NODES "${num_nodes}")
    configure_file(
      test/test_n_nodes.py.in
      test_n_nodes${target_suffix}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/test_n_nodes${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_n_nodes${target_suffix}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/test_n_nodes${target_suffix}_$<CONFIG>.py"
      TARGET test_n_nodes${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ${_ARG_UNPARSED_ARGUMENTS}
    )
    if(TEST test_n_nodes${target_suffix})
      set_tests_properties(test_n_nodes${target_suffix}
        PROPERTIES DEPENDS "node_with_name${target_suffix} node_check_names${target_suffix}"
      )
    endif()
  endmacro()

  # Macro for tests that trigger the shutdown of an executable based on particular console output,
  # then check the output of the executable against different console output.
  macro(custom_launch_test_executable_output test_name executable)

    cmake_parse_arguments(_ARG "" "LAUNCH_TEST_TEMPLATE" "" ${ARGN})
    if(NOT _ARG_LAUNCH_TEST_TEMPLATE)
      set(_ARG_LAUNCH_TEST_TEMPLATE test/test_executable_output.py.in)
    endif()

    set(TEST_NAME "${test_name}")
    set(TEST_EXECUTABLE "$<TARGET_FILE:${executable}>")
    set(TEST_EXECUTABLE_NAME "${executable}")
    set(TEST_EXECUTABLE_TRIGGER_SHUTDOWN_OUTPUT
      "${CMAKE_CURRENT_SOURCE_DIR}/test/expected_outputs/${test_name}__trigger_shutdown")
    set(TEST_EXECUTABLE_EXPECTED_OUTPUT
      "${CMAKE_CURRENT_SOURCE_DIR}/test/expected_outputs/${test_name}__expected_output")
    configure_file(
      ${_ARG_LAUNCH_TEST_TEMPLATE}
      ${test_name}${target_suffix}.py.configure
      @ONLY
    )
    file(GENERATE
      OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      INPUT "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}.py.configure"
    )
    add_launch_test(
      "${CMAKE_CURRENT_BINARY_DIR}/${test_name}${target_suffix}_$<CONFIG>.py"
      TARGET ${test_name}${target_suffix}
      ${_ARG_UNPARSED_ARGUMENTS}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ENV
        RMW_IMPLEMENTATION=${rmw_implementation}
    )
    if(TEST ${test_name}${target_suffix})
      set_tests_properties(${test_name}${target_suffix}
        PROPERTIES DEPENDS "${executable}${target_suffix}"
      )
    endif()
  endmacro()

  macro(cross_rmw_tests)
    # test node names cross rmw
    if(rmw_implementation1 STREQUAL rmw_implementation2)
      set(target_suffix "__${rmw_implementation1}")
    else()
      set(target_suffix "__${rmw_implementation1}__${rmw_implementation2}")
    endif()

    set(rmw_implementation1_is_fastrtps FALSE)
    set(rmw_implementation2_is_fastrtps FALSE)
    if(rmw_implementation1 MATCHES "(.*)fastrtps(.*)")
      set(rmw_implementation1_is_fastrtps TRUE)
    endif()
    if(rmw_implementation2 MATCHES "(.*)fastrtps(.*)")
      set(rmw_implementation2_is_fastrtps TRUE)
    endif()

    set(rmw_implementation1_is_connext FALSE)
    set(rmw_implementation2_is_connext FALSE)
    if(rmw_implementation1 MATCHES "(.*)connext(.*)")
      set(rmw_implementation1_is_connext TRUE)
    endif()
    if(rmw_implementation2 MATCHES "(.*)connext(.*)")
      set(rmw_implementation2_is_connext TRUE)
    endif()

    set(rmw_implementation1_is_cyclonedds FALSE)
    set(rmw_implementation2_is_cyclonedds FALSE)
    if(rmw_implementation1 MATCHES "(.*)cyclonedds(.*)")
      set(rmw_implementation1_is_cyclonedds TRUE)
    endif()
    if(rmw_implementation2 MATCHES "(.*)cyclonedds(.*)")
      set(rmw_implementation2_is_cyclonedds TRUE)
    endif()


    # Whitelist cross-vendor tests
    if(NOT (rmw_implementation1 STREQUAL rmw_implementation2))
      # TODO(sloretz) enable connext/cyclone/fastrtps when all three use 1 participant per context
      if(
        (rmw_implementation1_is_fastrtps AND rmw_implementation2_is_cyclonedds) OR
        (rmw_implementation1_is_cyclonedds AND rmw_implementation2_is_fastrtps) OR
        (rmw_implementation1_is_fastrtps AND rmw_implementation2_is_fastrtps)
      )
        # Whitelisted cross-vendor tests
        set(_crt_SKIP_TEST ${SKIP_TEST})
      else()
        # Default skip cross-vendor tests
        set(_crt_SKIP_TEST "SKIP_TEST")
      endif()
    else()
      # Same vendor tests always allowed
      set(_crt_SKIP_TEST ${SKIP_TEST})
    endif()

    custom_launch_test_two_executables(test_node_name
      node_with_name node_name_list
      ARGS1 "${rmw_implementation1}" ARGS2 "node_with_name_${rmw_implementation1}"
      RMW1 ${rmw_implementation1} RMW2 ${rmw_implementation2}
      TIMEOUT 15
      ${_crt_SKIP_TEST})
  endmacro()

  function(test_target)
    set(rmw_implementation_env_var RMW_IMPLEMENTATION=${rmw_implementation})

    ament_add_gtest_test(gtest_avoid_ros_namespace_conventions_qos
      TEST_NAME gtest_avoid_ros_namespace_conventions_qos${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 15
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_publisher
      TEST_NAME gtest_publisher${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 15
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_client_wait_for_service_shutdown
      TEST_NAME gtest_client_wait_for_service_shutdown${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_executor
      TEST_NAME gtest_executor${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_repeated_publisher_subscriber
      TEST_NAME gtest_repeated_publisher_subscriber${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 15
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_spin
      TEST_NAME gtest_spin${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 30
      ENV
        ${rmw_implementation_env_var}
    )

    # TODO(clalancette): Under load, the gtest_subscription__rmw_connextdds test fails deep in the
    # bowels of Connext; see https://github.com/ros2/rmw_connextdds/issues/136 for details.  Skip it
    # for now so we can keep CI green.
    ament_add_gtest_test(gtest_subscription
      TEST_NAME gtest_subscription${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )
    if("${rmw_implementation}" STREQUAL "rmw_connextdds")
      ament_add_test_label(gtest_subscription${target_suffix} xfail)
    endif()

    ament_add_gtest_test(gtest_multiple_service_calls
      TEST_NAME gtest_multiple_service_calls${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_timer
      TEST_NAME gtest_timer${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 30
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_timeout_subscriber
      TEST_NAME gtest_timeout_subscriber${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 30
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_intra_process
      TEST_NAME gtest_intra_process${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 15
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_multithreaded
      TEST_NAME gtest_multithreaded${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 90
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_local_parameters
      TEST_NAME gtest_local_parameters${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 300
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_services_in_constructor
      TEST_NAME gtest_services_in_constructor${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 30
      ENV
        ${rmw_implementation_env_var}
    )

    ament_add_gtest_test(gtest_waitable
      TEST_NAME gtest_waitable${target_suffix}
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      TIMEOUT 300
      ENV
        ${rmw_implementation_env_var}
    )

    # Parameter tests single implementation
    custom_launch_test_two_executables(test_parameter_server_cpp
      test_parameters_server_cpp test_remote_parameters_cpp
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )

    # Service tests single implementation
    custom_launch_test_two_executables(test_services_cpp
      test_services_server_cpp test_services_client_cpp
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )

    custom_launch_test_two_executables(test_client_scope_cpp
      test_client_scope_server_cpp test_client_scope_client_cpp
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )

    custom_launch_test_two_executables(test_client_scope_consistency_cpp
      test_client_scope_consistency_server_cpp test_client_scope_consistency_client_cpp
      TIMEOUT 60
      ENV
        ${rmw_implementation_env_var}
    )

    custom_launch_n_nodes(10
      TIMEOUT 15
    )

    # Note (dhood): signal handler tests are skipped on Windows because there is no opportunity
    # for signal handling once shutdown is triggered by launch_testing.
    set(SKIP_TEST "")
    if(WIN32)
      set(SKIP_TEST "SKIP_TEST")
    endif()
    # Test that a user-defined signal handler is called on interrupt:
    # after rclcpp::init has been called, but before rclcpp::shutdown has been called.
    custom_launch_test_executable_output(test_sigint_handler_before_shutdown
      test_sigint_handler
      TIMEOUT 30
      ${SKIP_TEST})

    # Test that a user-defined signal handler is restored after rclcpp::init and rclcpp::shutdown
    # have been called.
    custom_launch_test_executable_output(test_sigint_handler_after_shutdown
      test_sigint_handler
      TIMEOUT 30
      ${SKIP_TEST})

    # Test that a user-defined signal handler is called on interrupt:
    # after rclcpp::init has been called, but before rclcpp::shutdown has been called.
    custom_launch_test_executable_output(test_sigterm_handler_before_shutdown
      test_sigterm_handler
      LAUNCH_TEST_TEMPLATE test/test_sigterm.py.in
      TIMEOUT 30
      ${SKIP_TEST})

    # Test that a user-defined signal handler is restored after rclcpp::init and rclcpp::shutdown
    # have been called.
    custom_launch_test_executable_output(test_sigterm_handler_after_shutdown
      test_sigterm_handler
      LAUNCH_TEST_TEMPLATE test/test_sigterm.py.in
      TIMEOUT 30
      ${SKIP_TEST})

    # Test node names
    set(rmw_implementation1 "${rmw_implementation}")
    foreach(rmw_implementation2 ${rmw_implementations})
      cross_rmw_tests()
    endforeach()
  endfunction()

  get_available_rmw_implementations(rmw_implementations)
  call_for_each_rmw_implementation(test_target)
endif()  # BUILD_TESTING

# TODO should not install anything
ament_package()
