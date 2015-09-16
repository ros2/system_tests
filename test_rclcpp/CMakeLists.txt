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
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
    SKIP_INSTALL
  )

  # get the rmw implementations ahead of time
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  # create executables
  foreach(middleware_impl ${middleware_implementations})
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impl "${middleware_impl}")

    # test_repeated_publisher_subscriber
    ament_add_gtest(
      gtest_repeated_publisher_subscriber__${middleware_impl}
      "test/test_repeated_publisher_subscriber.cpp"
      TIMEOUT 15
    )
    if(TARGET gtest_repeated_publisher_subscriber__${middleware_impl})
      target_link_libraries(gtest_repeated_publisher_subscriber__${middleware_impl}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(gtest_repeated_publisher_subscriber__${middleware_impl}
        PUBLIC "RMW_IMPLEMENTATION=${middleware_impl}")
      add_dependencies(gtest_repeated_publisher_subscriber__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(gtest_repeated_publisher_subscriber__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(gtest_repeated_publisher_subscriber__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    # test_spin
    ament_add_gtest(
      gtest_spin__${middleware_impl}
      "test/test_spin.cpp"
      SKIP_LINKING_MAIN_LIBRARIES
      TIMEOUT 30
    )
    if(TARGET gtest_spin__${middleware_impl})
      target_link_libraries(gtest_spin__${middleware_impl}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(gtest_spin__${middleware_impl}
        PUBLIC "RMW_IMPLEMENTATION=${middleware_impl}")
      add_dependencies(gtest_spin__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(gtest_spin__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(gtest_spin__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    # test_subscription
    ament_add_gtest(
      gtest_subscription__${middleware_impl}
      "test/test_subscription.cpp"
      TIMEOUT 15
    )
    if(TARGET gtest_subscription__${middleware_impl})
      target_link_libraries(gtest_subscription__${middleware_impl}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(gtest_subscription__${middleware_impl}
        PUBLIC "RMW_IMPLEMENTATION=${middleware_impl}")
      add_dependencies(gtest_subscription__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(gtest_subscription__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(gtest_subscription__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    # test_timer
    ament_add_gtest(
      gtest_timer__${middleware_impl}
      "test/test_timer.cpp"
      SKIP_LINKING_MAIN_LIBRARIES
      TIMEOUT 30
    )
    if(TARGET gtest_timer__${middleware_impl})
      target_link_libraries(gtest_timer__${middleware_impl}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(gtest_timer__${middleware_impl}
        PUBLIC "RMW_IMPLEMENTATION=${middleware_impl}")
      add_dependencies(gtest_timer__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(gtest_timer__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(gtest_timer__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    # test_timeout_subscriber
    ament_add_gtest(
      gtest_timeout_subscriber__${middleware_impl}
      "test/test_timeout_subscriber.cpp"
      TIMEOUT 30
    )
    if(TARGET gtest_timeout_subscriber__${middleware_impl})
      target_link_libraries(gtest_timeout_subscriber__${middleware_impl}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(gtest_timeout_subscriber__${middleware_impl}
        PUBLIC "RMW_IMPLEMENTATION=${middleware_impl}")
      add_dependencies(gtest_timeout_subscriber__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(gtest_timeout_subscriber__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(gtest_timeout_subscriber__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    # test_intra_process
    ament_add_gtest(
      gtest_intra_process__${middleware_impl}
      "test/test_intra_process.cpp"
      TIMEOUT 15
    )
    if(TARGET gtest_intra_process__${middleware_impl})
      target_link_libraries(gtest_intra_process__${middleware_impl}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      target_compile_definitions(gtest_intra_process__${middleware_impl}
        PUBLIC "RMW_IMPLEMENTATION=${middleware_impl}")
      add_dependencies(gtest_intra_process__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(gtest_intra_process__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(gtest_intra_process__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    if(TARGET local_parameters_cpp__${middleware_impl})
      ament_add_gtest(local_parameters_cpp__${middleware_impl} SKIP_LINKING_MAIN_LIBRARIES "test/local_parameters.cpp")
      add_dependencies(local_parameters_cpp__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(local_parameters_cpp__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(local_parameters_cpp__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
    endif()

    if(TARGET test_remote_parameters_cpp__${middleware_impl})
      # Parameter tests single implementation
      add_executable(test_remote_parameters_cpp__${middleware_impl} "test/test_remote_parameters.cpp")
      target_include_directories(test_remote_parameters_cpp__${middleware_impl} PUBLIC ${GTEST_INCLUDE_DIRS})
      target_link_libraries(test_remote_parameters_cpp__${middleware_impl} ${GTEST_LIBRARIES})
      # the above could be replaced by a helper which won't register the gtest for running
      # ament_add_gtest_subtest(test_remote_parameters_cpp__${middleware_impl} "test/test_remote_parameters.cpp")
      add_dependencies(test_remote_parameters_cpp__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(test_remote_parameters_cpp__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(test_remote_parameters_cpp__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
      add_executable(test_parameters_server_cpp__${middleware_impl} "test/test_parameters_server.cpp")
      add_dependencies(test_parameters_server_cpp__${middleware_impl} ${PROJECT_NAME})
      rosidl_target_interfaces(test_parameters_server_cpp__${middleware_impl}
        ${PROJECT_NAME} ${typesupport_impl})
      ament_target_dependencies(test_parameters_server_cpp__${middleware_impl}
        "${middleware_impl}"
        "rclcpp")
      set(TEST_PARAMETERS_SERVER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_parameters_server_cpp__${middleware_impl}${test_executable_extension}")
      set(TEST_REMOTE_PARAMETERS_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_remote_parameters_cpp__${middleware_impl}${test_executable_extension}")
      configure_file(
        test/test_parameters_server.py.in
        test_parameters_server_cpp__${middleware_impl}.py
        @ONLY
      )
      ament_add_nose_test(parameter_server_cpp__${middleware_impl} "${CMAKE_CURRENT_BINARY_DIR}/test_parameters_server_cpp__${middleware_impl}.py" TIMEOUT 15)
        set_tests_properties(parameter_server_cpp__${middleware_impl} PROPERTIES DEPENDS "test_parameters_server_cpp__${middleware_impl} test_remote_parameters_cpp__${middleware_impl}")
    endif()

  endforeach()
endif()  # AMENT_ENABLE_TESTING

# TODO should not install anything
ament_package()
