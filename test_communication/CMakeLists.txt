cmake_minimum_required(VERSION 3.5)

project(test_communication)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall -Wextra -Wpedantic")
else()
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

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
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations2)
  foreach(rmw_implementation ${rmw_implementations2})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  function(custom_test target with_message_argument)
    if(with_message_argument)
      # adding test for each message type
      foreach(message_file ${message_files})
        set(SKIP_TEST "")
        get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
        # TODO(dirk-thomas) publishing Bounded/DynamicArrayPrimitives in OpenSplice is buggy
        # https://github.com/ros2/rclcpp/issues/219
        if(
          (TEST_MESSAGE_TYPE STREQUAL "BoundedArrayPrimitives" OR TEST_MESSAGE_TYPE STREQUAL "DynamicArrayPrimitives") AND
          rmw_implementation STREQUAL "rmw_opensplice_cpp"
        )
          set(SKIP_TEST "SKIP_TEST")
        endif()
        ament_add_test(
          "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
          COMMAND "$<TARGET_FILE:${target}>" "${TEST_MESSAGE_TYPE}"
          TIMEOUT 15
          GENERATE_RESULT_FOR_RETURN_CODE_ZERO
          APPEND_LIBRARY_DIRS "${append_library_dirs}"
          ENV
          RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
          RMW_IMPLEMENTATION=${rmw_implementation}
          ${SKIP_TEST})
        set_tests_properties(
          "${target}${target_suffix}__${TEST_MESSAGE_TYPE}"
          PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}>"
        )
      endforeach()
    else()
      ament_add_test(
        "${target}${target_suffix}"
        COMMAND "$<TARGET_FILE:${target}>"
        TIMEOUT 15
        GENERATE_RESULT_FOR_RETURN_CODE_ZERO
        APPEND_LIBRARY_DIRS "${append_library_dirs}"
        ENV
        RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
        RMW_IMPLEMENTATION=${rmw_implementation})
      set_tests_properties(
        "${target}${target_suffix}"
        PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}>"
      )
    endif()
  endfunction()

  function(custom_executable target)
    add_executable(${target} ${ARGN})
    rosidl_target_interfaces(${target}
      ${PROJECT_NAME} "rosidl_typesupport_cpp")
    ament_target_dependencies(${target}
      "rclcpp")
  endfunction()

  function(custom_test_c target)
    ament_add_gtest(
      "${target}${target_suffix}" ${ARGN}
      TIMEOUT 90
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation})
    target_link_libraries(${target}${target_suffix}
      ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
      ${_AMENT_EXPORT_LIBRARY_TARGETS})
    add_dependencies(${target}${target_suffix} ${PROJECT_NAME})
    rosidl_target_interfaces(${target}${target_suffix}
      ${PROJECT_NAME} "rosidl_typesupport_c")
    ament_target_dependencies(${target}${target_suffix}
      "rcl")
    set_tests_properties(
      ${target}${target_suffix}
      PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}${target_suffix}>"
    )
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
        OUTPUT "test_publisher_subscriber${test_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber${test_suffix}.py.configured"
      )

      set(SKIP_TEST "")
      # TODO(dirk-thomas) publishing Bounded/DynamicArrayPrimitives in OpenSplice is buggy
      # https://github.com/ros2/rclcpp/issues/219
      if(
        (TEST_MESSAGE_TYPE STREQUAL "BoundedArrayPrimitives" OR TEST_MESSAGE_TYPE STREQUAL "DynamicArrayPrimitives") AND
        rmw_implementation1 STREQUAL "rmw_opensplice_cpp"
      )
        set(SKIP_TEST "SKIP_TEST")
      endif()
      # TODO(mikaelarguedas) Simpler way to blacklist specific tests (e.g. regex matching)
      ament_add_nose_test(test_publisher_subscriber${test_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_publisher_subscriber${test_suffix}_$<CONFIG>.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
        PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
        APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../../rclpy"
        APPEND_LIBRARY_DIRS "${append_library_dirs}"
        TIMEOUT 15
        ${SKIP_TEST})
      set_tests_properties(
        test_publisher_subscriber${test_suffix}
        PROPERTIES DEPENDS "test_publisher_cpp__${rmw_implementation1};test_subscriber_cpp__${rmw_implementation2}"
      )
    endforeach()

    # test requester / replier
    set(SKIP_TEST "")

    # TODO different vendors can't talk to each other right now
    if(NOT rmw_implementation1 STREQUAL rmw_implementation2 AND
      (NOT(rmw_implementation1 MATCHES "(.*)connext(.*)" AND rmw_implementation2 MATCHES "(.*)connext(.*)"))
    )
      set(SKIP_TEST "SKIP_TEST")
    endif()

    # TODO(mikaelarguedas) connext doesn't support C services for now
    if((client_library1 STREQUAL "rclpy" OR client_library2 STREQUAL "rclpy") AND
      (rmw_implementation1 MATCHES "(.*)connext(.*)" OR rmw_implementation2 MATCHES "(.*)connext(.*)")
    )
      set(SKIP_TEST "SKIP_TEST")
    endif()

    if(WIN32 AND client_library1 STREQUAL "rclcpp" AND client_library2 STREQUAL "rclpy")
      set(SKIP_TEST "SKIP_TEST")
    endif()

    set(REQUESTER_RMW ${rmw_implementation1})
    set(REPLIER_RMW ${rmw_implementation2})
    foreach(service_file ${service_files})
      get_filename_component(TEST_SERVICE_TYPE "${service_file}" NAME_WE)
      set(test_suffix "__${TEST_SERVICE_TYPE}${suffix}")
      configure_file(
        test/test_requester_replier.py.in
        test_requester_replier${test_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_requester_replier${test_suffix}_$<CONFIG>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_requester_replier${test_suffix}.py.configured"
      )

      ament_add_nose_test(test_requester_replier${test_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_requester_replier${test_suffix}_$<CONFIG>.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
        PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
        APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../../rclpy"
        APPEND_LIBRARY_DIRS "${append_library_dirs}"
        TIMEOUT 30
        ${SKIP_TEST})
      set_tests_properties(
        test_requester_replier${test_suffix}
        PROPERTIES DEPENDS
          "test_requester_cpp__${rmw_implementation1};test_replier_cpp__${rmw_implementation2}"
      )
    endforeach()
  endmacro()

  macro(configure_template _client_library1 _client_library2)
    set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
    set(_client_library1 "${_client_library1}")
    set(_client_library2 "${_client_library2}")
    set(TEST_PUBLISHER_RCL "${_client_library1}")
    set(TEST_SUBSCRIBER_RCL "${_client_library2}")
    set(TEST_REQUESTER_RCL "${_client_library1}")
    set(TEST_REPLIER_RCL "${_client_library2}")

    if(_client_library1 STREQUAL _client_library2)
      set(suffix "__${_client_library1}")
    else()
      set(suffix "__${_client_library1}__${_client_library2}")
    endif()

    if(_client_library1 STREQUAL "rclpy" OR _client_library2 STREQUAL "rclpy")
      if(WIN32)
        if(CMAKE_BUILD_TYPE STREQUAL "Debug")
          set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
        endif()
      endif()
    endif()

    if(_client_library1 STREQUAL "rclpy")
      set(TEST_PUBLISHER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/publisher_py.py")
      set(TEST_REQUESTER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/requester_py.py")
    elseif(_client_library1 STREQUAL "rclcpp")
      set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_publisher_cpp>")
      set(TEST_REQUESTER_EXECUTABLE "$<TARGET_FILE:test_requester_cpp>")
    endif()

    if(_client_library2 STREQUAL "rclpy")
      set(TEST_SUBSCRIBER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/subscriber_py.py")
      set(TEST_REPLIER_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/replier_py.py")
    elseif(_client_library2 STREQUAL "rclcpp")
      set(TEST_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_subscriber_cpp>")
      set(TEST_REPLIER_EXECUTABLE "$<TARGET_FILE:test_replier_cpp>")
    endif()
  endmacro()


  macro(multi_client_libraries)
    # TODO(mikaelarguedas) Aggregate available rcl from ament index ?
    set(client_libraries "")
    list(APPEND client_libraries "rclpy")
    list(APPEND client_libraries "rclcpp")
    foreach(client_library1 ${client_libraries})
      foreach(client_library2 ${client_libraries})
        configure_template("${client_library1}" "${client_library2}")
        multi_targets()
      endforeach()
    endforeach()
  endmacro()

  macro(security_tests)
    # TODO(mikaelarguedas) extend this to connext once 5.2.4 released
    if(rmw_implementation STREQUAL "rmw_fastrtps_cpp")
      set(suffix "__${rmw_implementation}")
      set(PUBLISHER_RMW ${rmw_implementation})
      set(SUBSCRIBER_RMW ${rmw_implementation})
      set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
      set(_client_library1 "rclcpp")
      set(_client_library2 "rclcpp")
      set(TEST_PUBLISHER_RCL "${_client_library1}")
      set(TEST_SUBSCRIBER_RCL "${_client_library2}")

      set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_secure_publisher_cpp>")
      set(TEST_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_secure_subscriber_cpp>")

      set(PUBLISHER_NODE_NAME "publisher")
      set(SUBSCRIBER_NODE_NAME "subscriber")
      set(PUBLISHER_TOPIC_NAME "chatter")
      set(SUBSCRIBER_TOPIC_NAME "chatter")
      set(PUBLISHER_SHOULD_THROW "false")
      set(SUBSCRIBER_SHOULD_THROW "false")
      set(SUBSCRIBER_SHOULD_TIMEOUT "false")

      set(VALID_ROS_SECURE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/test/test_security_files/")

      # Test suite for communication without security
      set(PUBLISHER_ROS_ENABLE_SECURITY_LIST "false;false;false;true;false")
      set(SUBSCRIBER_ROS_ENABLE_SECURITY_LIST "false;false;false;false;true")
      set(PUBLISHER_ROS_SECURITY_STRATEGY_LIST "Enforce;garbage;garbage;Permissive;Garbage")
      set(SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST "Enforce;Permissive;Garbage;Garbage;Permissive")
      set(PUBLISHER_ROS_SECURE_ROOT_LIST "garbage;WHATEVER;${CMAKE_CURRENT_SOURCE_DIR}/test/test_security_files/;garbage;garbage")
      set(SUBSCRIBER_ROS_SECURE_ROOT_LIST "${CMAKE_CURRENT_SOURCE_DIR}/test/test_security_files;WHATEVER;garbage;garbage;garbage")

      # Test suite for working secured communication
      set(test_suite2_PUBLISHER_ROS_ENABLE_SECURITY_LIST "true;true;true;true")
      set(test_suite2_SUBSCRIBER_ROS_ENABLE_SECURITY_LIST "true;true;true;true")
      set(test_suite2_PUBLISHER_ROS_SECURITY_STRATEGY_LIST "Enforce;Enforce;Permissive;Permissive")
      set(test_suite2_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST "Enforce;Permissive;Enforce;Permissive")

      # Test suite for invalid configurations
      set(test_suite3_SUBSCRIBER_ROS_SECURE_ROOT_LIST "garbage;${VALID_ROS_SECURE_ROOT};${VALID_ROS_SECURE_ROOT};${VALID_ROS_SECURE_ROOT};${VALID_ROS_SECURE_ROOT}")
      set(test_suite3_SUBSCRIBER_NODE_NAMES "subscriber;subscriber2;subscriber_invalid_cert;subscriber_missing_key")

      set(n_non_secure_tests "")
      set(n_secure_communication_tests "")
      list(LENGTH PUBLISHER_ROS_ENABLE_SECURITY_LIST n_non_secure_tests)
      list(LENGTH test_suite2_PUBLISHER_ROS_ENABLE_SECURITY_LIST n_secure_communication_tests)
      list(LENGTH test_suite3_SUBSCRIBER_NODE_NAMES n_throwing_secure_tests)

      foreach(message_file ${message_files})
        get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
        # testing only one message type to avoid taking to much test time
        if(TEST_MESSAGE_TYPE STREQUAL "Empty")
          set(index 0)
          # configure all non_secure_tests
          while(index LESS ${n_non_secure_tests})
            # here we define all the environment variables needed for security
            list(GET PUBLISHER_ROS_ENABLE_SECURITY_LIST ${index} PUBLISHER_ROS_ENABLE_SECURITY)
            list(GET SUBSCRIBER_ROS_ENABLE_SECURITY_LIST ${index} SUBSCRIBER_ROS_ENABLE_SECURITY)
            list(GET PUBLISHER_ROS_SECURITY_STRATEGY_LIST ${index} PUBLISHER_ROS_SECURITY_STRATEGY)
            list(GET SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST ${index} SUBSCRIBER_ROS_SECURITY_STRATEGY)
            list(GET PUBLISHER_ROS_SECURE_ROOT_LIST ${index} PUBLISHER_ROS_SECURE_ROOT)
            list(GET SUBSCRIBER_ROS_SECURE_ROOT_LIST ${index} SUBSCRIBER_ROS_SECURE_ROOT)

            set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}__test_suite1_${index}")
            configure_file(
              test/test_secure_publisher_subscriber.py.in
              test_secure_publisher_subscriber${test_suffix}.py.configured
              @ONLY
            )
            file(GENERATE
              OUTPUT "test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
              INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}.py.configured"
            )
            math(EXPR index "${index} + 1")

            set(SKIP_TEST "")
            # TODO(mikaelarguedas) Simpler way to blacklist specific tests (e.g. regex matching)
            ament_add_nose_test(test_secure_publisher_subscriber${test_suffix}
              "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
              PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
              APPEND_LIBRARY_DIRS "${append_library_dirs}"
              TIMEOUT 15
              ${SKIP_TEST})
            set_tests_properties(
              test_secure_publisher_subscriber${test_suffix}
              PROPERTIES DEPENDS "test_secure_publisher_cpp__${rmw_implementation};test_secure_subscriber_cpp__${rmw_implementation}"
            )
          endwhile()

          set(index 0)
          set(PUBLISHER_ROS_SECURE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/test/test_security_files/")
          set(SUBSCRIBER_ROS_SECURE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/test/test_security_files/")
          # configure all non_secure_tests
          while( index LESS ${n_secure_communication_tests} )
            # here we define all the environment variables needed for security
            list(GET test_suite2_PUBLISHER_ROS_ENABLE_SECURITY_LIST ${index} PUBLISHER_ROS_ENABLE_SECURITY)
            list(GET test_suite2_SUBSCRIBER_ROS_ENABLE_SECURITY_LIST ${index} SUBSCRIBER_ROS_ENABLE_SECURITY)
            list(GET test_suite2_PUBLISHER_ROS_SECURITY_STRATEGY_LIST ${index} PUBLISHER_ROS_SECURITY_STRATEGY)
            list(GET test_suite2_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST ${index} SUBSCRIBER_ROS_SECURITY_STRATEGY)

            set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}__test_suite2_${index}")
            configure_file(
              test/test_secure_publisher_subscriber.py.in
              test_secure_publisher_subscriber${test_suffix}.py.configured
              @ONLY
            )
            file(GENERATE
              OUTPUT "test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
              INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}.py.configured"
            )
            math(EXPR index "${index} + 1")

            set(SKIP_TEST "")
            # TODO(mikaelarguedas) Simpler way to blacklist specific tests (e.g. regex matching)
            ament_add_nose_test(test_secure_publisher_subscriber${test_suffix}
              "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
              PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
              APPEND_LIBRARY_DIRS "${append_library_dirs}"
              TIMEOUT 15
              ${SKIP_TEST})
            set_tests_properties(
              test_secure_publisher_subscriber${test_suffix}
              PROPERTIES DEPENDS "test_secure_publisher_cpp__${rmw_implementation};test_secure_subscriber_cpp__${rmw_implementation}"
            )
          endwhile()

          set(index 0)
          # configure all trhowing tests
          set(PUBLISHER_ROS_ENABLE_SECURITY "false")
          set(SUBSCRIBER_ROS_ENABLE_SECURITY "true")
          set(PUBLISHER_ROS_SECURITY_STRATEGY "Garbage")
          set(SUBSCRIBER_ROS_SECURITY_STRATEGY "Enforce")
          set(PUBLISHER_ROS_SECURE_ROOT "GARBAGEGARBAGE")
          set(SUBSCRIBER_NODE_NAME "subscriber")
          set(PUBLISHER_NODE_NAME "publisher")
          set(PUBLISHER_SHOULD_THROW "false")
          set(SUBSCRIBER_SHOULD_THROW "true")
          # set(SUBSCRIBER_SHOULD_TIMEOUT "false")

          while( index LESS ${n_throwing_secure_tests} )
            list(GET test_suite3_SUBSCRIBER_NODE_NAMES ${index} SUBSCRIBER_NODE_NAME)
            list(GET test_suite3_SUBSCRIBER_ROS_SECURE_ROOT_LIST ${index} SUBSCRIBER_ROS_SECURE_ROOT)
            set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}__test_suite3_${index}")
            configure_file(
              test/test_secure_publisher_subscriber.py.in
              test_secure_publisher_subscriber${test_suffix}.py.configured
              @ONLY
            )
            file(GENERATE
              OUTPUT "test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
              INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}.py.configured"
            )
            math(EXPR index "${index} + 1")

            set(SKIP_TEST "")
            # TODO(mikaelarguedas) Simpler way to blacklist specific tests (e.g. regex matching)
            ament_add_nose_test(test_secure_publisher_subscriber${test_suffix}
              "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
              PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
              APPEND_LIBRARY_DIRS "${append_library_dirs}"
              TIMEOUT 15
              ${SKIP_TEST})
            set_tests_properties(
              test_secure_publisher_subscriber${test_suffix}
              PROPERTIES DEPENDS "test_secure_publisher_cpp__${rmw_implementation};test_secure_subscriber_cpp__${rmw_implementation}"
            )
          endwhile()
        endif()
      endforeach()
    endif()
  endmacro()

  # publisher combined with a subscriber
  custom_executable(test_publisher_subscriber_cpp
    "test/test_publisher_subscriber.cpp")
  # subcription valid data
  custom_executable(test_subscription_valid_data_cpp
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
  # executables secure publisher / subscriber
  custom_executable(test_secure_publisher_cpp
    "test/test_secure_publisher.cpp")
  custom_executable(test_secure_subscriber_cpp
    "test/test_secure_subscriber.cpp")

  macro(targets)
    custom_test_c(test_messages_c
      "test/test_messages_c.cpp")

    security_tests()

    # publisher combined with a subscriber
    custom_test(test_publisher_subscriber_cpp TRUE)
    # subcription valid data
    custom_test(test_subscription_valid_data_cpp FALSE)

    set(rmw_implementation1 "${rmw_implementation}")
    set(target_suffix1 "${target_suffix}")

    foreach(rmw_implementation2 ${rmw_implementations2})
      multi_client_libraries()
    endforeach()
  endmacro()

  set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
  if(WIN32)
    set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
  endif()

  call_for_each_rmw_implementation(targets)
endif()  # BUILD_TESTING

install(FILES test/__init__.py test/message_fixtures.py
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

ament_auto_package()
