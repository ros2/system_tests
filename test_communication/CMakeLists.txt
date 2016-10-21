cmake_minimum_required(VERSION 3.5)

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
    "msg/BoundedArrayNested.msg"
    "msg/BoundedArrayPrimitives.msg"
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

  function(custom_test target with_message_argument)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

    add_executable(${target}${target_suffix} ${ARGN})
    add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
    foreach(typesupport_impl ${typesupport_impls})
      rosidl_target_interfaces(${target}${target_suffix}
        ${PROJECT_NAME} ${typesupport_impl})
    endforeach()
    ament_target_dependencies(${target}${target_suffix}
      "rclcpp${target_suffix}")
    if(with_message_argument)
      # adding test for each message type
      foreach(message_file ${message_files})
        get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
        # TODO(dirk-thomas) publishing Bounded/DynamicArrayPrimitives in OpenSplice is buggy
        # https://github.com/ros2/rclcpp/issues/219
        if((NOT TEST_MESSAGE_TYPE STREQUAL "BoundedArrayPrimitives" AND NOT TEST_MESSAGE_TYPE STREQUAL "DynamicArrayPrimitives") OR NOT rmw_implementation STREQUAL "rmw_opensplice_cpp")
          ament_add_test(
            "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
            COMMAND "$<TARGET_FILE:${target}${target_suffix}>" "${TEST_MESSAGE_TYPE}"
            TIMEOUT 15
            GENERATE_RESULT_FOR_RETURN_CODE_ZERO
            ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation})
          set_tests_properties(
            "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
            PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}${target_suffix}>"
          )
        endif()
      endforeach()
    else()
      ament_add_test(
        "${target}${target_suffix}"
        COMMAND "$<TARGET_FILE:${target}${target_suffix}>"
        TIMEOUT 15
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        ENV RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation})
      set_tests_properties(
        "${target}${target_suffix}"
        PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}${target_suffix}>"
      )
    endif()
  endfunction()

  function(custom_executable target)
    # get typesupport of rmw implementation to include / link against the corresponding interfaces
    get_rmw_typesupport(typesupport_impls "${rmw_implementation}" LANGUAGE "cpp")

    add_executable(${target}${target_suffix} ${ARGN})
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
      set(suffix "${suffix}__${rmw_implementation1}")
    else()
      set(suffix "${suffix}__${rmw_implementation1}__${rmw_implementation2}")
    endif()
    set(PUBLISHER_RMW ${rmw_implementation1})
    set(SUBSCRIBER_RMW ${rmw_implementation2})
    foreach(message_file ${message_files})
      get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
      set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}")
      configure_file(
        test/test_publisher_subscriber.py.in
        test_publisher_subscriber${test_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_publisher_subscriber${test_suffix}_$<CONFIGURATION>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber${test_suffix}.py.configured"
      )

      # TODO(dirk-thomas) publishing Bounded/DynamicArrayPrimitives in OpenSplice is buggy
      # https://github.com/ros2/rclcpp/issues/219
      if((NOT TEST_MESSAGE_TYPE STREQUAL "BoundedArrayPrimitives" AND NOT TEST_MESSAGE_TYPE STREQUAL "DynamicArrayPrimitives") OR NOT rmw_implementation1 STREQUAL "rmw_opensplice_cpp")
        ament_add_nose_test(test_publisher_subscriber${test_suffix}
          "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber${test_suffix}_$<CONFIGURATION>.py"
          WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
          PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
          APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../../rclpy"
          APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
          APPEND_LIBRARY_DIRS "${_append_library_dirs}"
          TIMEOUT 15)
        set_tests_properties(
          test_publisher_subscriber${test_suffix}
          PROPERTIES DEPENDS "test_publisher_cpp__${rmw_implementation1};test_subscriber_cpp__${rmw_implementation2}"
        )
      endif()
    endforeach()

    # TODO(mikaelarguedas) modify this once python services are implemented
    if(rcl1 STREQUAL "rclcpp" AND rcl2 STREQUAL "rclcpp")
      # TODO different vendors can't talk to each other right now
      if(
        rmw_implementation1 STREQUAL rmw_implementation2 OR
        (rmw_implementation1 STREQUAL "rmw_connext_cpp" AND rmw_implementation2 STREQUAL "rmw_connext_dynamic_cpp") OR
        (rmw_implementation1 STREQUAL "rmw_connext_dynamic_cpp" AND rmw_implementation2 STREQUAL "rmw_connext_cpp")
      )
      # test requester / replier
      set(TEST_REQUESTER_EXECUTABLE "$<TARGET_FILE:test_requester_cpp__${rmw_implementation1}>")
      set(REQUESTER_RMW ${rmw_implementation1})
      set(TEST_REPLIER_EXECUTABLE "$<TARGET_FILE:test_replier_cpp__${rmw_implementation2}>")
      set(REPLIER_RMW ${rmw_implementation2})
      foreach(service_file ${service_files})
        get_filename_component(TEST_SERVICE_TYPE "${service_file}" NAME_WE)
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

        ament_add_nose_test(
          requester_replier_cpp${test_suffix}
          "${CMAKE_CURRENT_BINARY_DIR}/test_requester_replier_cpp${test_suffix}_$<CONFIGURATION>.py"
          TIMEOUT 30
        )
        set_tests_properties(
          requester_replier_cpp${test_suffix}
          PROPERTIES DEPENDS
            "test_requester_cpp__${rmw_implementation1};test_replier_cpp__${rmw_implementation2}"
        )
      endforeach()
      endif()
    endif()
  endmacro()

  macro(multi_rcl)
    # test publish / subscribe messages
    set(rcls "")
    # TODO(mikaelarguedas) Aggregate available rcl from ament index ?
    list(APPEND rcls "rclpy")
    list(APPEND rcls "rclcpp")
    foreach(rcl1 ${rcls})
      set(TEST_PUBLISHER_RCL ${rcl1})
      if(rcl1 STREQUAL "rclcpp")
        set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_publisher_cpp__${rmw_implementation1}>")
      elseif(rcl1 STREQUAL "rclpy")
        set(_append_library_dirs "")
        if(WIN32)
          set(_append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>")
          if(CMAKE_BUILD_TYPE STREQUAL "Debug")
            set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
          else()
            set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
          endif()
        endif()
        set(TEST_PUBLISHER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/publisher_py.py")
      endif()
      foreach(rcl2 ${rcls})
        set(TEST_SUBSCRIBER_RCL ${rcl2})
        if(rcl1 STREQUAL rcl2)
          set(suffix "__${rcl1}")
        else()
          set(suffix "__${rcl1}__${rcl2}")
        endif()
        if(rcl2 STREQUAL "rclcpp")
          set(TEST_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_subscriber_cpp__${rmw_implementation2}>")
        elseif(rcl2 STREQUAL "rclpy")
          set(TEST_SUBSCRIBER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/subscriber_py.py")
          set(_append_library_dirs "")
          if(WIN32)
            set(_append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>")
            if(CMAKE_BUILD_TYPE STREQUAL "Debug")
              set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
            else()
              set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
            endif()
          endif()
        endif()
        if(NOT (rmw_implementation1 STREQUAL "rmw_connext_dynamic_cpp" OR rmw_implementation2 STREQUAL "rmw_connext_dynamic_cpp"))
            multi_targets()
        endif()
      endforeach()
    endforeach()
  endmacro()

  macro(targets)
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

#    # publisher combined with a subscriber
#    custom_test(test_publisher_subscriber_cpp
#      TRUE
#      "test/test_publisher_subscriber.cpp")
#    # subcription valid data
#    custom_test(test_subscription_valid_data_cpp
#      FALSE
#      "test/test_subscription_valid_data.cpp")

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
        multi_rcl()
      endforeach()
  endmacro()

  call_for_each_rmw_implementation(targets)
endif()  # BUILD_TESTING

# TODO should not install anything
ament_package()
