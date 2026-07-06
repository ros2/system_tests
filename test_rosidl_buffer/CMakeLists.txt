cmake_minimum_required(VERSION 3.20)
project(test_rosidl_buffer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_buffer REQUIRED)
find_package(rosidl_buffer_backend REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_typesupport_fastrtps_cpp REQUIRED)
find_package(std_msgs REQUIRED)

# Generate the descriptor + payload interfaces used by the backend and nodes.
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ByteArray.msg"
  "msg/ByteArrayList.msg"
  "msg/TestBufferDescriptor.msg"
)

rosidl_get_typesupport_target(_cpp_ts ${PROJECT_NAME} "rosidl_typesupport_cpp")
rosidl_get_typesupport_target(_fastrtps_ts ${PROJECT_NAME} "rosidl_typesupport_fastrtps_cpp")

# Header-only target exposing TestBufferImpl<T>. Deliberately separated from
# the pluginlib SHARED lib below: executables link this INTERFACE target only,
# which avoids the double-free at shutdown caused by linking a pluginlib-loaded
# plugin library directly (class_loader ProxyExec + __cxa_finalize both run dtors).
add_library(${PROJECT_NAME}_impl INTERFACE)
target_include_directories(${PROJECT_NAME}_impl INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(${PROJECT_NAME}_impl INTERFACE
  ${_cpp_ts}
  rosidl_buffer::rosidl_buffer
  rosidl_runtime_cpp::rosidl_runtime_cpp
)

# Pluginlib-exported BufferBackend implementation (SHARED).
add_library(${PROJECT_NAME}_backend_plugin SHARED
  src/test_buffer_backend_plugin.cpp
)
target_include_directories(${PROJECT_NAME}_backend_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_definitions(${PROJECT_NAME}_backend_plugin
  PRIVATE "TEST_ROSIDL_BUFFER_BUILDING_DLL")
target_link_libraries(${PROJECT_NAME}_backend_plugin PUBLIC
  ament_cmake_ros_core::ament_ros_defaults
  ${_cpp_ts}
  ${_fastrtps_ts}
  pluginlib::pluginlib
  rmw::rmw
  rosidl_buffer::rosidl_buffer
  rosidl_buffer_backend::rosidl_buffer_backend
  rosidl_runtime_cpp::rosidl_runtime_cpp
  rosidl_typesupport_cpp::rosidl_typesupport_cpp
  rosidl_typesupport_fastrtps_cpp::rosidl_typesupport_fastrtps_cpp
)

pluginlib_export_plugin_description_file(
  rosidl_buffer_backend test_buffer_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(
  TARGETS ${PROJECT_NAME}_backend_plugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  FILES test_buffer_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ---------------------------------------------------------------------------
  # Tier 1: standalone gtest, no RMW involved.
  # ---------------------------------------------------------------------------
  ament_add_gtest(test_buffer_backend_unit test/test_buffer_backend_unit.cpp)
  if(TARGET test_buffer_backend_unit)
    target_link_libraries(test_buffer_backend_unit
      ament_cmake_ros_core::ament_ros_defaults
      ${PROJECT_NAME}_backend_plugin
      ${PROJECT_NAME}_impl
      rosidl_buffer::rosidl_buffer
      rosidl_buffer_backend::rosidl_buffer_backend
    )
  endif()

  # ---------------------------------------------------------------------------
  # Tier 2: launch tests, gated by RMWs that have end-to-end buffer-backend
  # support. Today that is only rmw_fastrtps_cpp — extend this list as other
  # RMWs land full backend support.
  # ---------------------------------------------------------------------------
  set(_buffer_backend_capable_rmws rmw_fastrtps_cpp)

  find_package(rmw_implementation_cmake REQUIRED)
  get_available_rmw_implementations(_available_rmws)

  set(_has_capable_rmw FALSE)
  foreach(rmw_impl ${_available_rmws})
    if(rmw_impl IN_LIST _buffer_backend_capable_rmws)
      set(_has_capable_rmw TRUE)
      find_package("${rmw_impl}" REQUIRED)
    endif()
  endforeach()

  if(_has_capable_rmw)
    find_package(launch_testing_ament_cmake REQUIRED)

    # Pub/sub executables. Link only the header-only impl target and the
    # generated cpp typesupport — NEVER the plugin SHARED lib (see note above).
    add_executable(test_backend_publisher src/test_backend_publisher.cpp)
    target_link_libraries(test_backend_publisher
      ament_cmake_ros_core::ament_ros_defaults
      ${PROJECT_NAME}_impl
      rclcpp::rclcpp
      ${std_msgs_TARGETS}
    )

    add_executable(test_backend_subscriber src/test_backend_subscriber.cpp)
    target_link_libraries(test_backend_subscriber
      ament_cmake_ros_core::ament_ros_defaults
      ${PROJECT_NAME}_impl
      rclcpp::rclcpp
      ${std_msgs_TARGETS}
    )

    add_executable(test_backend_nested_msgs_publisher
      src/test_backend_nested_msgs_publisher.cpp)
    target_link_libraries(test_backend_nested_msgs_publisher
      ament_cmake_ros_core::ament_ros_defaults
      ${PROJECT_NAME}_impl
      rclcpp::rclcpp
      ${std_msgs_TARGETS}
    )

    add_executable(test_backend_nested_msgs_subscriber
      src/test_backend_nested_msgs_subscriber.cpp)
    target_link_libraries(test_backend_nested_msgs_subscriber
      ament_cmake_ros_core::ament_ros_defaults
      ${PROJECT_NAME}_impl
      rclcpp::rclcpp
      ${std_msgs_TARGETS}
    )

    install(
      TARGETS
        test_backend_publisher
        test_backend_subscriber
        test_backend_nested_msgs_publisher
        test_backend_nested_msgs_subscriber
      DESTINATION lib/${PROJECT_NAME}
    )

    # Register one launch test per capable RMW for each scenario.
    foreach(rmw_impl ${_buffer_backend_capable_rmws})
      if(NOT rmw_impl IN_LIST _available_rmws)
        continue()
      endif()

      if(rmw_impl STREQUAL "rmw_fastrtps_cpp")
        add_launch_test(test/test_test_to_test_fastrtps.py
          TARGET test_test_to_test__${rmw_impl}
          TIMEOUT 60
        )
        add_launch_test(test/test_test_to_cpu_fastrtps.py
          TARGET test_test_to_cpu__${rmw_impl}
          TIMEOUT 60
        )
        add_launch_test(test/test_nested_msgs_fastrtps.py
          TARGET test_nested_msgs__${rmw_impl}
          TIMEOUT 60
        )
      endif()
    endforeach()
  endif()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
