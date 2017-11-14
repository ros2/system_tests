cmake_minimum_required(VERSION 3.5)

project(test_security)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND MSVC)
  # /bigobj is needed to avoid error C1128:
  #   https://msdn.microsoft.com/en-us/library/8578y171.aspx
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

option(SECURITY "Activate security" OFF)

if(BUILD_TESTING)
  # No security test on services for now
  find_package(ament_cmake REQUIRED)
  find_package(test_msgs REQUIRED)

  ament_index_get_resource(interface_files "rosidl_interfaces" "test_msgs")
  string(REPLACE "\n" ";" interface_files "${interface_files}")

  set(message_files "")

  # set(service_files "")
  foreach(interface_file ${interface_files})
    string_ends_with("${interface_file}" ".msg" is_message)
    if(is_message)
      list(APPEND message_files "${interface_file}")
      continue()
    endif()
    # string_ends_with("${interface_file}" ".srv" is_service)
    # if(is_service)
    #   list(APPEND service_files "${interface_file}")
    #   continue()
    # endif()
  endforeach()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations)
  foreach(rmw_implementation ${rmw_implementations})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  function(custom_executable target)
    add_executable(${target} ${ARGN})
    ament_target_dependencies(${target}
      "rclcpp" "test_msgs")
  endfunction()

  function(custom_security_test_c target)
    ament_add_gtest(
      "${target}${target_suffix}" ${ARGN}
      TIMEOUT 10
      APPEND_LIBRARY_DIRS "${append_library_dirs}"
      ENV
      RCL_ASSERT_RMW_ID_MATCHES=${rmw_implementation}
      RMW_IMPLEMENTATION=${rmw_implementation}
      ROS_SECURITY_ROOT_DIRECTORY="${VALID_ROS_SECURITY_ROOT_DIRECTORY}"
    )
    if(TARGET ${target}${target_suffix})
      target_link_libraries(${target}${target_suffix}
        ${_AMENT_EXPORT_ABSOLUTE_LIBRARIES}
        ${_AMENT_EXPORT_LIBRARY_TARGETS})
      ament_target_dependencies(${target}${target_suffix}
        "rcl")
      set_tests_properties(
        ${target}${target_suffix}
        PROPERTIES REQUIRED_FILES "$<TARGET_FILE:${target}${target_suffix}>"
      )
    endif()
  endfunction()

  macro(security_tests)
    set(suffix "__${rmw_implementation}")
    set(PUBLISHER_RMW ${rmw_implementation})
    set(SUBSCRIBER_RMW ${rmw_implementation})
    # Not testing across client libraries for now
    set(TEST_PUBLISHER_RCL "rclcpp")
    set(TEST_SUBSCRIBER_RCL "rclcpp")
    set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_secure_publisher_cpp>")
    set(TEST_SUBSCRIBER_EXECUTABLE "$<TARGET_FILE:test_secure_subscriber_cpp>")

    # Test suite for communication without security
    set(non_secure_comm_PUBLISHER_ROS_SECURITY_ENABLE_LIST "false;false;false;true;false")
    set(non_secure_comm_SUBSCRIBER_ROS_SECURITY_ENABLE_LIST "false;false;false;false;true")
    set(non_secure_comm_PUBLISHER_ROS_SECURITY_STRATEGY_LIST "Enforce;garbage;garbage;Permissive;Garbage")
    set(non_secure_comm_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST "Enforce;Permissive;Garbage;Garbage;Permissive")
    set(non_secure_comm_PUBLISHER_ROS_SECURITY_ROOT_DIRECTORY_LIST "garbage;WHATEVER;${VALID_ROS_SECURITY_ROOT_DIRECTORY};garbage;garbage")
    set(SUBSCRIBER_ROS_SECURITY_ROOT_DIRECTORY_LIST "${VALID_ROS_SECURITY_ROOT_DIRECTORY};WHATEVER;garbage;garbage;garbage")

    # Test suite for secured communication
    set(secure_comm_PUBLISHER_ROS_SECURITY_ENABLE_LIST "true;true;true;true")
    set(secure_comm_SUBSCRIBER_ROS_SECURITY_ENABLE_LIST "true;true;true;true")
    set(secure_comm_PUBLISHER_ROS_SECURITY_STRATEGY_LIST "Enforce;Enforce;Permissive;Permissive")
    set(secure_comm_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST "Enforce;Permissive;Enforce;Permissive")

    # Test suite for one node with security and the second without
    set(not_connecting_PUBLISHER_ROS_SECURITY_ENABLE_LIST "false;true")
    set(not_connecting_SUBSCRIBER_ROS_SECURITY_ENABLE_LIST "true;false")
    set(not_connecting_PUBLISHER_ROS_SECURITY_STRATEGY_LIST "Permissive;Enforce")
    set(not_connecting_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST "Enforce;Permissive")

    list(LENGTH non_secure_comm_PUBLISHER_ROS_SECURITY_ENABLE_LIST n_non_secure_tests)
    list(LENGTH secure_comm_PUBLISHER_ROS_SECURITY_ENABLE_LIST n_secure_communication_tests)
    list(LENGTH not_connecting_PUBLISHER_ROS_SECURITY_ENABLE_LIST n_not_connecting_tests)

    foreach(message_file ${message_files})
      get_filename_component(TEST_MESSAGE_TYPE "${message_file}" NAME_WE)
      # TODO(mikaelarguedas) test only few message types to avoid taking to much test time
      if(TEST_MESSAGE_TYPE STREQUAL "Empty" OR TEST_MESSAGE_TYPE STREQUAL "DynamicArrayNested")
        set(index 0)
        # configure all non secure communication tests
        set(SUBSCRIBER_SHOULD_TIMEOUT "false")
        while(index LESS ${n_non_secure_tests})
          # here we define all the variables needed for security template expansion
          list(GET non_secure_comm_PUBLISHER_ROS_SECURITY_ENABLE_LIST ${index} PUBLISHER_ROS_SECURITY_ENABLE)
          list(GET SUBSCRIBER_ROS_SECURITY_ENABLE_LIST ${index} SUBSCRIBER_ROS_SECURITY_ENABLE)
          list(GET PUBLISHER_ROS_SECURITY_STRATEGY_LIST ${index} PUBLISHER_ROS_SECURITY_STRATEGY)
          list(GET SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST ${index} SUBSCRIBER_ROS_SECURITY_STRATEGY)
          list(GET PUBLISHER_ROS_SECURITY_ROOT_DIRECTORY_LIST ${index} PUBLISHER_ROS_SECURITY_ROOT_DIRECTORY)
          list(GET SUBSCRIBER_ROS_SECURITY_ROOT_DIRECTORY_LIST ${index} SUBSCRIBER_ROS_SECURITY_ROOT_DIRECTORY)

          set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}__non_secure_comm_${index}")
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

          ament_add_pytest_test(test_secure_publisher_subscriber${test_suffix}
            "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
            APPEND_LIBRARY_DIRS "${append_library_dirs}"
            TIMEOUT 20
          )
          set_tests_properties(
            test_secure_publisher_subscriber${test_suffix}
            PROPERTIES DEPENDS "test_secure_publisher_cpp__${rmw_implementation};test_secure_subscriber_cpp__${rmw_implementation}"
          )
        endwhile()

        set(index 0)
        set(SUBSCRIBER_SHOULD_TIMEOUT "false")
        set(PUBLISHER_ROS_SECURITY_ROOT_DIRECTORY "${VALID_ROS_SECURITY_ROOT_DIRECTORY}")
        set(SUBSCRIBER_ROS_SECURITY_ROOT_DIRECTORY "${VALID_ROS_SECURITY_ROOT_DIRECTORY}")
        # configure all secure communication tests
        while(index LESS ${n_secure_communication_tests})
          # here we define all the variables needed for security template expansion
          list(GET secure_comm_PUBLISHER_ROS_SECURITY_ENABLE_LIST ${index} PUBLISHER_ROS_SECURITY_ENABLE)
          list(GET secure_comm_SUBSCRIBER_ROS_SECURITY_ENABLE_LIST ${index} SUBSCRIBER_ROS_SECURITY_ENABLE)
          list(GET secure_comm_PUBLISHER_ROS_SECURITY_STRATEGY_LIST ${index} PUBLISHER_ROS_SECURITY_STRATEGY)
          list(GET secure_comm_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST ${index} SUBSCRIBER_ROS_SECURITY_STRATEGY)

          set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}__secure_comm_${index}")
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

          ament_add_pytest_test(test_secure_publisher_subscriber${test_suffix}
            "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
            APPEND_LIBRARY_DIRS "${append_library_dirs}"
            TIMEOUT 20
          )
          set_tests_properties(
            test_secure_publisher_subscriber${test_suffix}
            PROPERTIES DEPENDS "test_secure_publisher_cpp__${rmw_implementation};test_secure_subscriber_cpp__${rmw_implementation}"
          )
        endwhile()

        set(index 0)
        set(PUBLISHER_ROS_SECURITY_ROOT_DIRECTORY "${VALID_ROS_SECURITY_ROOT_DIRECTORY}")
        set(SUBSCRIBER_ROS_SECURITY_ROOT_DIRECTORY "${VALID_ROS_SECURITY_ROOT_DIRECTORY}")
        set(SUBSCRIBER_SHOULD_TIMEOUT "true")
        # configure all not connecting tests
        while(index LESS ${n_not_connecting_tests})
          # here we define all the variables needed for security template expansion
          list(GET not_connecting_PUBLISHER_ROS_SECURITY_ENABLE_LIST ${index} PUBLISHER_ROS_SECURITY_ENABLE)
          list(GET not_connecting_SUBSCRIBER_ROS_SECURITY_ENABLE_LIST ${index} SUBSCRIBER_ROS_SECURITY_ENABLE)
          list(GET not_connecting_PUBLISHER_ROS_SECURITY_STRATEGY_LIST ${index} PUBLISHER_ROS_SECURITY_STRATEGY)
          list(GET not_connecting_SUBSCRIBER_ROS_SECURITY_STRATEGY_LIST ${index} SUBSCRIBER_ROS_SECURITY_STRATEGY)

          set(test_suffix "__${TEST_MESSAGE_TYPE}${suffix}__secure_not_connecting_${index}")
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

          ament_add_pytest_test(test_secure_publisher_subscriber${test_suffix}
            "${CMAKE_CURRENT_BINARY_DIR}/test_secure_publisher_subscriber${test_suffix}_$<CONFIG>.py"
            APPEND_LIBRARY_DIRS "${append_library_dirs}"
            TIMEOUT 20
          )
          set_tests_properties(
            test_secure_publisher_subscriber${test_suffix}
            PROPERTIES DEPENDS "test_secure_publisher_cpp__${rmw_implementation};test_secure_subscriber_cpp__${rmw_implementation}"
          )
        endwhile()
      endif()
    endforeach()
  endmacro()

  macro(targets)
    set(VALID_ROS_SECURITY_ROOT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/test_security_files")

    # TODO(mikaelarguedas) only connext and fastrtps support DDS-Security for now
    if(rmw_implementation STREQUAL "rmw_connext_cpp" OR rmw_implementation STREQUAL "rmw_fastrtps_cpp")
      custom_security_test_c(test_security_nodes_c
        "test/test_invalid_secure_node_creation_c.cpp")
      security_tests()
    endif()
  endmacro()

  if(SECURITY)
    # executables secure publisher / subscriber
    custom_executable(test_secure_publisher_cpp
      "test/test_secure_publisher.cpp")
    custom_executable(test_secure_subscriber_cpp
      "test/test_secure_subscriber.cpp")

    set(append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}")
    if(WIN32)
      set(append_library_dirs "${append_library_dirs}/$<CONFIG>")
    endif()

    call_for_each_rmw_implementation(targets)
  endif()
endif()  # BUILD_TESTING

ament_auto_package()
