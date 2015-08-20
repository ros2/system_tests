cmake_minimum_required(VERSION 2.8.3)

project(test_rclcpp)

if(NOT WIN32)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
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

  endforeach()
endif()  # AMENT_ENABLE_TESTING

# TODO should not install anything
ament_package()
