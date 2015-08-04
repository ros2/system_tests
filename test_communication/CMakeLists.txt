cmake_minimum_required(VERSION 2.8.3)

project(test_communication)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
endif()

find_package(ament_cmake REQUIRED)

if(AMENT_ENABLE_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(message_files
    "msg/Builtins.msg"
    "msg/DynamicArrayNested.msg"
    "msg/DynamicArrayPrimitives.msg"
    "msg/Empty.msg"
    "msg/Nested.msg"
    "msg/Primitives.msg"
    "msg/StaticArrayNested.msg"
    "msg/StaticArrayPrimitives.msg"
  )
  set(service_files
    "srv/Empty.srv"
    "srv/Primitives.srv"
  )
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
    ${service_files}
    DEPENDENCIES builtin_interfaces
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
  get_available_rmw_implementations(middleware_implementations)
  foreach(middleware_impl ${middleware_implementations})
    find_package("${middleware_impl}" REQUIRED)
  endforeach()

  # create executables
  foreach(middleware_impl ${middleware_implementations})
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impl "${middleware_impl}")

    # executables publisher combined with a subscriber
    add_executable(test_publisher_subscriber_cpp__${middleware_impl} "test/test_publisher_subscriber.cpp")
    target_link_libraries(test_publisher_subscriber_cpp__${middleware_impl}
      ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
      ${_AMENT_EXPORT_LIBRARY_TARGETS})
    add_dependencies(test_publisher_subscriber_cpp__${middleware_impl} ${PROJECT_NAME})
    rosidl_target_interfaces(test_publisher_subscriber_cpp__${middleware_impl}
      ${PROJECT_NAME} ${typesupport_impl})
    ament_target_dependencies(test_publisher_subscriber_cpp__${middleware_impl}
      "${middleware_impl}"
      "rclcpp")
    # register executable as a test
    get_target_property(target_path test_publisher_subscriber_cpp__${middleware_impl} RUNTIME_OUTPUT_DIRECTORY)
    if(NOT target_path)
      set(target_path "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    # testing this with a single message type should be enough
    ament_add_test(
      "test_publisher_subscriber_cpp__${middleware_impl}"
      COMMAND "${target_path}/test_publisher_subscriber_cpp__${middleware_impl}" empty
      TIMEOUT 15
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    )
    set_tests_properties(
      test_publisher_subscriber_cpp__${middleware_impl}
      PROPERTIES REQUIRED_FILES "${target_path}/test_publisher_subscriber_cpp__${middleware_impl}"
    )

    # executables publisher / subscriber
    add_executable(test_publisher_cpp__${middleware_impl} "test/test_publisher.cpp")
    target_link_libraries(test_publisher_cpp__${middleware_impl}
      ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
      ${_AMENT_EXPORT_LIBRARY_TARGETS})
    add_dependencies(test_publisher_cpp__${middleware_impl} ${PROJECT_NAME})
    rosidl_target_interfaces(test_publisher_cpp__${middleware_impl}
      ${PROJECT_NAME} ${typesupport_impl})
    ament_target_dependencies(test_publisher_cpp__${middleware_impl}
      "${middleware_impl}"
      "rclcpp")

    add_executable(test_subscriber_cpp__${middleware_impl} "test/test_subscriber.cpp")
    add_dependencies(test_subscriber_cpp__${middleware_impl} ${PROJECT_NAME})
    rosidl_target_interfaces(test_subscriber_cpp__${middleware_impl}
      ${PROJECT_NAME} ${typesupport_impl})
    ament_target_dependencies(test_subscriber_cpp__${middleware_impl}
      "${middleware_impl}"
      "rclcpp")

    # executables requester / replier
    add_executable(test_requester_cpp__${middleware_impl} "test/test_requester.cpp")
    add_dependencies(test_requester_cpp__${middleware_impl} ${PROJECT_NAME})
    rosidl_target_interfaces(test_requester_cpp__${middleware_impl}
      ${PROJECT_NAME} ${typesupport_impl})
    ament_target_dependencies(test_requester_cpp__${middleware_impl}
      "${middleware_impl}"
      "rclcpp")

    add_executable(test_replier_cpp__${middleware_impl} "test/test_replier.cpp")
    add_dependencies(test_replier_cpp__${middleware_impl} ${PROJECT_NAME})
    rosidl_target_interfaces(test_replier_cpp__${middleware_impl}
      ${PROJECT_NAME} ${typesupport_impl})
    ament_target_dependencies(test_replier_cpp__${middleware_impl}
      "${middleware_impl}"
      "rclcpp")
  endforeach()

  # create tests for each combination of rmw implementation
  foreach(middleware_impl1 ${middleware_implementations})
    foreach(middleware_impl2 ${middleware_implementations})
      # test publish / subscribe messages
      if(middleware_impl1 STREQUAL middleware_impl2)
        set(middleware_impls_suffix "__${middleware_impl1}")
      else()
        set(middleware_impls_suffix "__${middleware_impl1}__${middleware_impl2}")
      endif()
      set(TEST_PUBLISHER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_publisher_cpp__${middleware_impl1}${test_executable_extension}")
      set(TEST_SUBSCRIBER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_subscriber_cpp__${middleware_impl2}${test_executable_extension}")
      foreach(message_file ${message_files})
        get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
        string(TOLOWER "${TEST_MESSAGE_TYPE}" TEST_MESSAGE_TYPE)
        set(test_suffix "__${TEST_MESSAGE_TYPE}${middleware_impls_suffix}")
        configure_file(
          test/test_publisher_subscriber.py.in
          test_publisher_subscriber_cpp${test_suffix}.py
          @ONLY
        )

        ament_add_nose_test(publisher_subscriber_cpp${test_suffix} "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber_cpp${test_suffix}.py" TIMEOUT 15)
        set_tests_properties(
          publisher_subscriber_cpp${test_suffix}
          PROPERTIES DEPENDS "test_publisher_cpp__${middleware_impl1};test_subscriber_cpp__${middleware_impl2}"
        )
      endforeach()

      # TODO different vendors can't talk to each other right now
      if((NOT "${middleware_impl1} " STREQUAL "rmw_opensplice_cpp " AND NOT "${middleware_impl2} " STREQUAL "rmw_opensplice_cpp ") OR "${middleware_impl1} " STREQUAL "${middleware_impl2} ")
      # test requester / replier
      set(TEST_REQUESTER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_requester_cpp__${middleware_impl1}${test_executable_extension}")
      set(TEST_REPLIER_EXECUTABLE "${CMAKE_CURRENT_BINARY_DIR}${test_executable_subfolder}/test_replier_cpp__${middleware_impl2}${test_executable_extension}")
      foreach(service_file ${service_files})
        get_filename_component(TEST_SERVICE_TYPE "${service_file}" NAME_WE)
        string(TOLOWER "${TEST_SERVICE_TYPE}" TEST_SERVICE_TYPE)
        set(test_suffix "__${TEST_SERVICE_TYPE}${middleware_impls_suffix}")
        configure_file(
          test/test_requester_replier.py.in
          test_requester_replier_cpp${test_suffix}.py
          @ONLY
        )

        ament_add_nose_test(requester_replier_cpp${test_suffix} "${CMAKE_CURRENT_BINARY_DIR}/test_requester_replier_cpp${test_suffix}.py" TIMEOUT 15)
        set_tests_properties(
          requester_replier_cpp${test_suffix}
          PROPERTIES DEPENDS "test_requester_cpp__${middleware_impl1};test_replier_cpp__${middleware_impl2}"
        )
      endforeach()
      endif()
    endforeach()
  endforeach()
endif()  # AMENT_ENABLE_TESTING

# TODO should not install anything
ament_package()
