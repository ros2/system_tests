cmake_minimum_required(VERSION 3.5)

project(test_rclpy)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # get the rmw implementations ahead of time
  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(rmw_implementations)
  foreach(rmw_implementation ${rmw_implementations})
    find_package("${rmw_implementation}" REQUIRED)
  endforeach()

  set(message_files
    "msg/DynamicArrayPrimitives.msg"
    "msg/Primitives.msg"
  )
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${message_files}
    SKIP_INSTALL
  )

  macro(rostopic_test)
    # test rostopic list
    find_package(rmw_implementation_cmake REQUIRED)
    set(TEST_PUBLISHER_PY_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/publisher_py.py")
    set(TEST_PUBLISHER_EXECUTABLE "$<TARGET_FILE:test_publisher_cpp__${rmw_implementation}>")
    set(TEST_ROSTOPIC_EXECUTABLE "${CMAKE_CURRENT_SOURCE_DIR}/test/rostopic_list.py")
    get_rmw_typesupport(typesupport_c_impl "${rmw_implementation}" LANGUAGE "c")
    set(TEST_RMW "${rmw_implementation}")
    if(NOT typesupport_c_impl STREQUAL "")
      set(test_suffix "__${rmw_implementation}")
      configure_file(
        test/test_rostopic_list.py.in
        test_rostopic_list${test_suffix}.py.configured
        @ONLY
      )
      file(GENERATE
        OUTPUT "test_rostopic_list${test_suffix}_$<CONFIGURATION>.py"
        INPUT "${CMAKE_CURRENT_BINARY_DIR}/test_rostopic_list${test_suffix}.py.configured"
      )

      set(_append_library_dirs "")
      if(WIN32)
        set(_append_library_dirs "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIGURATION>")
      endif()
      if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
        set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
      else()
        set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
      endif()
      ament_add_nose_test(test_rostopic_list${test_suffix}
        "${CMAKE_CURRENT_BINARY_DIR}/test_rostopic_list${test_suffix}_$<CONFIGURATION>.py"
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py"
        PYTHON_EXECUTABLE "${_PYTHON_EXECUTABLE}"
        APPEND_ENV PYTHONPATH="${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_py:${CMAKE_CURRENT_SOURCE_DIR}:${CMAKE_CURRENT_BINARY_DIR}/../../rclpy"
        APPEND_ENV AMENT_PREFIX_PATH=${ament_index_build_path}
        APPEND_LIBRARY_DIRS "${_append_library_dirs}"
        TIMEOUT 15)
    endif()
  endmacro()

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
      "rclcpp${target_suffix}"
    )
  endfunction()

  macro(targets)
    get_rclcpp_information("${rmw_implementation}" "rclcpp${target_suffix}")

    custom_executable(test_publisher_cpp
      "test/test_publisher.cpp")
    rostopic_test()
  endmacro()

  call_for_each_rmw_implementation(targets)
endif()  # BUILD_TESTING

# TODO should not install anything
ament_package()
