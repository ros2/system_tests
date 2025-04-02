^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.20.3 (2025-04-02)
-------------------

0.20.2 (2024-04-16)
-------------------

0.20.1 (2024-03-27)
-------------------

0.20.0 (2023-12-26)
-------------------
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

0.16.1 (2023-05-11)
-------------------

0.16.0 (2023-04-28)
-------------------

0.15.1 (2023-04-11)
-------------------

0.15.0 (2023-02-13)
-------------------
* Update the system tests to C++17. (`#510 <https://github.com/ros2/system_tests/issues/510>`_)
* [rolling] Update maintainers - 2022-11-07 (`#509 <https://github.com/ros2/system_tests/issues/509>`_)
* Contributors: Audrow Nash, Chris Lalancette

0.14.0 (2022-09-13)
-------------------

0.13.0 (2022-05-04)
-------------------

0.12.3 (2022-04-05)
-------------------

0.12.2 (2022-03-28)
-------------------

0.12.1 (2022-01-13)
-------------------

0.12.0 (2021-11-18)
-------------------
* Updated maintainers (`#489 <https://github.com/ros2/system_tests/issues/489>`_)
* Add changelogs (`#473 <https://github.com/ros2/system_tests/issues/473>`_)
* Contributors: Aditya Pande, Ivan Santiago Paunovic

0.11.1 (2021-04-26)
-------------------

0.11.0 (2021-04-06)
-------------------

0.10.0 (2021-03-18)
-------------------
* Update maintainers. (`#450 <https://github.com/ros2/system_tests/issues/450>`_)
* Enable -Wall, -Wextra, and -Wpedantic. (`#447 <https://github.com/ros2/system_tests/issues/447>`_)
* Contributors: Audrow Nash, Jacob Perron

0.9.1 (2020-07-06)
------------------

0.9.0 (2020-06-04)
------------------

0.8.0 (2019-11-20)
------------------
* 0.8.0
* Promote special CLI rules to flags. (`#385 <https://github.com/ros2/system_tests/issues/385>`_)
* Use of -r/--remap flags where appropriate. (`#384 <https://github.com/ros2/system_tests/issues/384>`_)
* Adapt to '--ros-args ... [--]'-based ROS args extraction. (`#381 <https://github.com/ros2/system_tests/issues/381>`_)
  * Use --ros-args in test_cli package tests.
  * Use --ros-args in test_cli_remapping package tests.
* Remove the test_cli dependency on launch. (`#375 <https://github.com/ros2/system_tests/issues/375>`_)
  It never uses it, so we don't need it.
* [test_cli] don't fail the build if the test is missing. (`#371 <https://github.com/ros2/system_tests/issues/371>`_)
* Contributors: Chris Lalancette, Michael Carroll, Michel Hidalgo, Mikael Arguedas

0.7.1 (2019-05-29)
------------------
* Update to use new parameter option names. (`#370 <https://github.com/ros2/system_tests/issues/370>`_)
* Contributors: William Woodall

0.7.0 (2019-05-20)
------------------
* Disable long term flaky test test_params_yaml. (`#369 <https://github.com/ros2/system_tests/issues/369>`_)
* Declaring initial parameters. (`#358 <https://github.com/ros2/system_tests/issues/358>`_)
* Merge pull request `#356 <https://github.com/ros2/system_tests/issues/356>`_ from ros2/issue/321_enhance_parameter_api
* Using new parameter API; allowing undeclared parameters.
* Read only parameters. (`#278 <https://github.com/ros2/system_tests/issues/278>`_)
  * Expect declared parameters + use_sim_time
  * Replace create_parameter with declare_parameter
  * Expect declared parameters + use_sim_time
  * Replace create_parameter with declare_parameter
  * Fixup node constructor now that we have NodeOptions
  * Cleanup test_parameters_server.cpp
  * Silence warnings for tests of deprecated methods
  * Remove redundant test (now lives in rclcpp's test_node.cpp)
  * Fixup tests
  * Extend deprecation warning suppression to support Windows too
  * Use option to allow declaring of parameters via yaml file for test
* Fixes a failing test introduced by the switch to array.array. (`#341 <https://github.com/ros2/system_tests/issues/341>`_)
* Contributors: Dirk Thomas, Juan Ignacio Ubeira, Shane Loretz

0.6.0 (2018-12-14)
------------------
* Test parameter behavior for rclpy nodes. (`#293 <https://github.com/ros2/system_tests/issues/293>`_)
* Use debug python executable on windows. (`#281 <https://github.com/ros2/system_tests/issues/281>`_)
  * Use debug python executable on windows
  * Get python debug executable from pythonextra
* Test initializing parameters from command line. (`#274 <https://github.com/ros2/system_tests/issues/274>`_)
  * Add test_cli package with tests for __params:= argument
* Contributors: Shane Loretz, Steven! Ragnarök

0.4.0 (2017-12-08)
------------------
