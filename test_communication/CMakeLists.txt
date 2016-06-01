cmake_minimum_required(VERSION 2.8.3)

project(test_communication)

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

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations2)
  foreach(rmw_implementation ${rmw_implementations2})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

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
  set(other_message_files
    "msg/FieldsWithSameType.msg"
    "msg/UInt32.msg"
  )
  set(service_files
    "srv/Empty.srv"
    "srv/Primitives.srv"
  )
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
    ${other_message_files}
    ${service_files}
    DEPENDENCIES builtin_interfaces
    SKIP_INSTALL
  )

  function(custom_test target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

    add_executable(${target}${target_suffix} ${ARGN})
    target_link_libraries(${target}${target_suffix}
      ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
      ${_AMENT_EXPORT_LIBRARY_TARGETS})
    add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
    foreach(typesupport_impl ${typesupport_impls})
      rosidl_target_interfaces(${target}${target_suffix}
        ${PROJECT_NAME} ${typesupport_impl})
    endforeach()
    ament_target_dependencies(${target}${target_suffix}
      "rclcpp${target_suffix}")
    # register executable as a test
    get_target_property(target_path ${target}${target_suffix}
      RUNTIME_OUTPUT_DIRECTORY)
    if(NOT target_path)
      set(target_path "${CMAKE_CURRENT_BINARY_DIR}")
    endif()
    # testing this with a single message type should be enough
    ament_add_test(
      "${target}${target_suffix}"
      COMMAND "${target_path}/${target}${target_suffix}" empty
      TIMEOUT 15
      GENERATE_RESULT_FOR_RETURN_CODE_ZERO
      ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation})
    set_tests_properties(
      ${target}${target_suffix}
      PROPERTIES REQUIRED_FILES "${target_path}/${target}${target_suffix}"
    )
  endfunction()

  function(custom_executable target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

    add_executable(${target}${target_suffix} ${ARGN})
    target_link_libraries(${target}${target_suffix}
      ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
      ${_AMENT_EXPORT_LIBRARY_TARGETS})
    add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
    foreach(typesupport_impl ${typesupport_impls})
      rosidl_target_interfaces(${target}${target_suffix}
        ${PROJECT_NAME} ${typesupport_impl})
    endforeach()
    ament_target_dependencies(${target}${target_suffix}
      "rclcpp${target_suffix}")
  endfunction()

  macro(multi_targets)
    # test publish / subscribe messages
    if(rmw_implementation1 STREQUAL rmw_implementation2)
      set(suffix "__${rmw_implementation1}")
    else()
      set(suffix "__${rmw_implementation1}__${rmw_implementation2}")
    endif()
    set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_publisher_cpp__${rmw_implementation1}>")
    set(TEST_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_subscriber_cpp__${rmw_implementation2}>")
    foreach(message_file ${message_files})
      get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
      string(TOLOWER "${TEST_MESSAGE_TYPE}" TEST_MESSAGE_TYPE)
      set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}")
      configure_file(
        test/test_publisher_subscriber.py.in
        test_publisher_subscriber_cpp${test_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_publisher_subscriber_cpp${test_suffix}_$<CONFIGURATION>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber_cpp${test_suffix}.py.configured"
      )

      ament_add_nose_test(publisher_subscriber_cpp${test_suffix} "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber_cpp${test_suffix}_$<CONFIGURATION>.py" TIMEOUT 15)
      set_tests_properties(
        publisher_subscriber_cpp${test_suffix}
        PROPERTIES DEPENDS "test_publisher_cpp__${rmw_implementation1};test_subscriber_cpp__${rmw_implementation2}"
      )
    endforeach()

    # TODO different vendors can't talk to each other right now
    if((NOT "${rmw_implementation1} " STREQUAL "rmw_opensplice_cpp " AND NOT "${rmw_implementation2} " STREQUAL "rmw_opensplice_cpp ") OR "${rmw_implementation1} " STREQUAL "${rmw_implementation2} ")
    # test requester / replier
    set(TEST_REQUESTER_EXECUTABLE "$<TARGET_FILE:test_requester_cpp__${rmw_implementation1}>")
    set(TEST_REPLIER_EXECUTABLE "$<TARGET_FILE:test_replier_cpp__${rmw_implementation2}>")
    foreach(service_file ${service_files})
      get_filename_component(TEST_SERVICE_TYPE "${service_file}" NAME_WE)
      string(TOLOWER "${TEST_SERVICE_TYPE}" TEST_SERVICE_TYPE)
      set(test_suffix "__${TEST_SERVICE_TYPE}${suffix}")
      configure_file(
        test/test_requester_replier.py.in
        test_requester_replier_cpp${test_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_requester_replier_cpp${test_suffix}_$<CONFIGURATION>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_requester_replier_cpp${test_suffix}.py.configured"
      )

      ament_add_nose_test(requester_replier_cpp${test_suffix} "${CMAKE_CURRENT_BINARY_DIR}/test_requester_replier_cpp${test_suffix}_$<CONFIGURATION>.py" TIMEOUT 15)
      set_tests_properties(
        requester_replier_cpp${test_suffix}
        PROPERTIES DEPENDS "test_requester_cpp__${rmw_implementation1};test_replier_cpp__${rmw_implementation2}"
      )
    endforeach()
    endif()
  endmacro()

  macro(targets)
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

    # publisher combined with a subscriber
    custom_test(test_publisher_subscriber_cpp
      "test/test_publisher_subscriber.cpp")
    # subcription valid data
    custom_test(test_subscription_valid_data_cpp
      "test/test_subscription_valid_data.cpp")

    # executables publisher / subscriber
    custom_executable(test_publisher_cpp
      "test/test_publisher.cpp")
    custom_executable(test_subscriber_cpp
      "test/test_subscriber.cpp")
    # executables requester / replier
    custom_executable(test_requester_cpp
      "test/test_requester.cpp")
    custom_executable(test_replier_cpp
      "test/test_replier.cpp")

    set(rmw_implementation1 "${rmw_implementation}")
    set(target_suffix1 "${target_suffix}")

    foreach(rmw_implementation2 ${rmw_implementations2})
      set(target_suffix2 "__${rmw_implementation2}")
      multi_targets()
    endforeach()
  endmacro()

  call_for_each_rmw_implementation(targets)
endif()  # BUILD_TESTING

# TODO should not install anything
ament_package()
