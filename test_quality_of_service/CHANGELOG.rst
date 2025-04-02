^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_quality_of_service
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.20.3 (2025-04-02)
-------------------
* Use rmw_event_type_is_supported to skip tests (backport `#563 <https://github.com/ros2/system_tests/issues/563>`_) (`#564 <https://github.com/ros2/system_tests/issues/564>`_)
  * Use rmw_event_type_is_supported to skip tests (`#563 <https://github.com/ros2/system_tests/issues/563>`_)
  (cherry picked from commit 8d0b7dea57044f14139e9354366f9c1871a47746)
* Contributors: mergify[bot]

0.20.2 (2024-04-16)
-------------------

0.20.1 (2024-03-27)
-------------------

0.20.0 (2023-12-26)
-------------------
* Cleanup header includes in test_quality_of_service. (`#533 <https://github.com/ros2/system_tests/issues/533>`_)
* Switch to target_link_libraries everywhere. (`#532 <https://github.com/ros2/system_tests/issues/532>`_)
* Contributors: Chris Lalancette

0.19.0 (2023-10-04)
-------------------

0.18.0 (2023-09-07)
-------------------

0.17.1 (2023-08-21)
-------------------

0.17.0 (2023-07-11)
-------------------
* Fix test QoS on macOS by moving qos_utilities.cpp to the four tests; fixes `#517 <https://github.com/ros2/system_tests/issues/517>`_ (`#518 <https://github.com/ros2/system_tests/issues/518>`_)
* Contributors: PhDittmann

0.16.1 (2023-05-11)
-------------------

0.16.0 (2023-04-28)
-------------------

0.15.1 (2023-04-11)
-------------------
* Fix ODR errors with gtest (`#514 <https://github.com/ros2/system_tests/issues/514>`_)
* Avoid flaky test (`#513 <https://github.com/ros2/system_tests/issues/513>`_)
* Contributors: Chen Lihui, Chris Lalancette, methylDragon

0.15.0 (2023-02-13)
-------------------
* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`_)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.14.0 (2022-09-13)
-------------------
* Pass rclcpp::QoS to create_service (`#507 <https://github.com/ros2/system_tests/issues/507>`_)
* Pass rclcpp::QoS to create_client (`#506 <https://github.com/ros2/system_tests/issues/506>`_)
* Remove Werror from test_quality_of_service. (`#503 <https://github.com/ros2/system_tests/issues/503>`_)
* Revert "Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`_)" (`#504 <https://github.com/ros2/system_tests/issues/504>`_)
* Replace deprecated spin_until_future_complete with spin_until_complete (`#499 <https://github.com/ros2/system_tests/issues/499>`_)
* Contributors: Chris Lalancette, Hubert Liberacki, Shane Loretz, William Woodall

0.13.0 (2022-05-04)
-------------------
* Add tests for 'best available' QoS policies (`#501 <https://github.com/ros2/system_tests/issues/501>`_)
* Contributors: Jacob Perron

0.12.3 (2022-04-05)
-------------------

0.12.2 (2022-03-28)
-------------------

0.12.1 (2022-01-13)
-------------------
* Update maintainers to Aditya Pande and Shane Loretz (`#491 <https://github.com/ros2/system_tests/issues/491>`_)
* Contributors: Audrow Nash

0.12.0 (2021-11-18)
-------------------
* Updated maintainers (`#489 <https://github.com/ros2/system_tests/issues/489>`_)
* Fix deprecated subscriber callback warnings (`#483 <https://github.com/ros2/system_tests/issues/483>`_)
* Add changelogs (`#473 <https://github.com/ros2/system_tests/issues/473>`_)
* Contributors: Abrar Rahman Protyasha, Aditya Pande, Ivan Santiago Paunovic

0.11.1 (2021-04-26)
-------------------

0.11.0 (2021-04-06)
-------------------

0.10.0 (2021-03-18)
-------------------
* Add support for rmw_connextdds. (`#463 <https://github.com/ros2/system_tests/issues/463>`_)
* Run QoS tests. (`#441 <https://github.com/ros2/system_tests/issues/441>`_)
* Update maintainers. (`#450 <https://github.com/ros2/system_tests/issues/450>`_)
* Contributors: Andrea Sorbini, Jacob Perron, Michel Hidalgo

0.9.1 (2020-07-06)
------------------

0.9.0 (2020-06-04)
------------------
* Code style only: wrap after open parenthesis if not in one line. (`#397 <https://github.com/ros2/system_tests/issues/397>`_)
* Contributors: Dirk Thomas

0.8.0 (2019-11-20)
------------------
* 0.8.0
* Remove broken constructors. (`#389 <https://github.com/ros2/system_tests/issues/389>`_)
* Contributors: Dan Rose, Michael Carroll

0.7.1 (2019-05-29)
------------------
* QoS System Tests. (`#347 <https://github.com/ros2/system_tests/issues/347>`_)
  * Added new Quality of Service system test package
  Added simple test for liveliness
  * Added deadline system test
  Added lifespan system test
  * Added linting fixes
  * Added documentation
  Added minor liveliness test cleanup
  * Fix CMake warnings
  * Added base class for test nodes
  Added implementation headers and files
  Addressed review comments
  * Added simple deadline test
  Renamed implementing classes to avoid namespace confusion
  * Addressed review comments
  Fixed bugs in fixture bringup and teardown
  Fixed base class issues
  * Fix compiler errors
  * Update to be compatible with latest rclcpp API
  * Removes unused find_package for pytest.
  * Fix windows compiler warnings
  * Replace rclcpp::get_logger() with Node::get_logger()
* Contributors: Devin Bonnie

0.7.0 (2019-05-20)
------------------

0.6.0 (2018-12-14)
------------------

0.4.0 (2017-12-08)
------------------
