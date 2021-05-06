^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_cli
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.11.1 (2021-04-26)
-------------------

0.11.0 (2021-04-06)
-------------------

0.10.0 (2021-03-18)
-------------------
* Update maintainers (`#450 <https://github.com/ros2/system_tests/issues/450>`_)
* Enable -Wall, -Wextra, and -Wpedantic (`#447 <https://github.com/ros2/system_tests/issues/447>`_)
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
* Adapt to '--ros-args ... [--]'-based ROS args extraction (`#381 <https://github.com/ros2/system_tests/issues/381>`_)
  * Use --ros-args in test_cli package tests.
  * Use --ros-args in test_cli_remapping package tests.
* Remove the test_cli dependency on launch. (`#375 <https://github.com/ros2/system_tests/issues/375>`_)
  It never uses it, so we don't need it.
* [test_cli] don't fail the build if the test is missing (`#371 <https://github.com/ros2/system_tests/issues/371>`_)
* Contributors: Chris Lalancette, Michael Carroll, Michel Hidalgo, Mikael Arguedas

0.7.1 (2019-05-29)
------------------
* update to use new parameter option names (`#370 <https://github.com/ros2/system_tests/issues/370>`_)
* Contributors: William Woodall

0.7.0 (2019-05-20)
------------------
* disable long term flaky test test_params_yaml (`#369 <https://github.com/ros2/system_tests/issues/369>`_)
* Declaring initial parameters. (`#358 <https://github.com/ros2/system_tests/issues/358>`_)
* Merge pull request `#356 <https://github.com/ros2/system_tests/issues/356>`_ from ros2/issue/`#321 <https://github.com/ros2/system_tests/issues/321>`__enhance_parameter_api
  Using new parameter API: allowing undeclared parameters.
* Using new parameter API; allowing undeclared parameters.
* Read only parameters (`#278 <https://github.com/ros2/system_tests/issues/278>`_)
  * Expect declared parameters + use_sim_time
  * replace create_parameter with declare_parameter
  * Expect declared parameters + use_sim_time
  * replace create_parameter with declare_parameter
  * fixup node constructor now that we have NodeOptions
  * cleanup test_parameters_server.cpp
  * silence warnings for tests of deprecated methods
  * remove redundant test (now lives in rclcpp's test_node.cpp)
  * fixup tests
  * extend deprecation warning suppression to support Windows too
  * use option to allow declaring of parameters via yaml file for test
* fixes a failing test introduced by the switch to array.array (`#341 <https://github.com/ros2/system_tests/issues/341>`_)
* Contributors: Dirk Thomas, Juan Ignacio Ubeira, Shane Loretz

0.6.0 (2018-12-14)
------------------
* Test parameter behavior for rclpy nodes (`#293 <https://github.com/ros2/system_tests/issues/293>`_)
* Use debug python executable on windows (`#281 <https://github.com/ros2/system_tests/issues/281>`_)
  * use debug python executable on windows
  * get python debug executable from pythonextra
* Test initializing parameters from command line (`#274 <https://github.com/ros2/system_tests/issues/274>`_)
  * Add test_cli package with tests for __params:= argument
* Contributors: Shane Loretz, Steven! Ragnarök

0.4.0 (2017-12-08)
------------------
