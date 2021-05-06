^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_quality_of_service
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.11.1 (2021-04-26)
-------------------

0.11.0 (2021-04-06)
-------------------

0.10.0 (2021-03-18)
-------------------
* Add support for rmw_connextdds (`#463 <https://github.com/ros2/system_tests/issues/463>`_)
* Run QoS tests. (`#441 <https://github.com/ros2/system_tests/issues/441>`_)
  * Fix deadline QoS tests.
  * Use longer deadlines for rmw_connext_cpp.
  * Fix lifespan QoS test.
  * Use longer lifespans for rmw_connext_cpp.
  * Explain different QoS durations.
  * Cope with discovery noise in lifespan test.
  * Adjust lifespan test again to deal discovery.
* Update maintainers (`#450 <https://github.com/ros2/system_tests/issues/450>`_)
* Contributors: Andrea Sorbini, Jacob Perron, Michel Hidalgo

0.9.1 (2020-07-06)
------------------

0.9.0 (2020-06-04)
------------------
* code style only: wrap after open parenthesis if not in one line (`#397 <https://github.com/ros2/system_tests/issues/397>`_)
* Contributors: Dirk Thomas

0.8.0 (2019-11-20)
------------------
* 0.8.0
* Remove broken constructors (`#389 <https://github.com/ros2/system_tests/issues/389>`_)
* Contributors: Dan Rose, Michael Carroll

0.7.1 (2019-05-29)
------------------
* QoS System Tests (`#347 <https://github.com/ros2/system_tests/issues/347>`_)
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
  * fix compiler errors
  * update to be compatible with latest rclcpp API
  * Removes unused find_package for pytest.
  * fix windows compiler warnings
  * replace rclcpp::get_logger() with Node::get_logger()
* Contributors: Devin Bonnie

0.7.0 (2019-05-20)
------------------

0.6.0 (2018-12-14)
------------------

0.4.0 (2017-12-08)
------------------
