^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package test_communication
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------

0.11.1 (2021-04-26)
-------------------

0.11.0 (2021-04-06)
-------------------

0.10.0 (2021-03-18)
-------------------
* Add support for rmw_connextdds (`#463 <https://github.com/ros2/system_tests/issues/463>`_)
* Kill off the ros2 daemon before running tests.
* Remove Opensplice from test_communication.
* Make TestMessageSerialization robust to missed messages (`#456 <https://github.com/ros2/system_tests/issues/456>`_)
  Though unlikely, I think it's possible for the first couple messages to be missed since we directly
  start publishing after creating the subscription. If this happens, it will throw off the expected
  value based on the counter. I think this is what was responsible for the reported test failure in `#451 <https://github.com/ros2/system_tests/issues/451>`_.
  This change accounts for possible missed messages by setting the counter to the value in the first received message.
* Add corresponding rclcpp::shutdown (`#455 <https://github.com/ros2/system_tests/issues/455>`_)
* Update maintainers (`#450 <https://github.com/ros2/system_tests/issues/450>`_)
* Contributors: Andrea Sorbini, Chris Lalancette, Jacob Perron, Stephen Brawner

0.9.1 (2020-07-06)
------------------

0.9.0 (2020-06-04)
------------------
* avoid new deprecations (`#426 <https://github.com/ros2/system_tests/issues/426>`_)
  * avoid new deprecations
  * avoid more deprecations
* use serilaized message in callback (`#427 <https://github.com/ros2/system_tests/issues/427>`_)
* fix CMake warning about using uninitialized variables (`#425 <https://github.com/ros2/system_tests/issues/425>`_)
* disabled Connext-CycloneDDS WString tests (`#421 <https://github.com/ros2/system_tests/issues/421>`_)
* rename rosidl_generator_c namespace to rosidl_runtime_c (`#416 <https://github.com/ros2/system_tests/issues/416>`_)
* add options to selectively ignore single/multi RMW tests (`#403 <https://github.com/ros2/system_tests/issues/403>`_)
* code style only: wrap after open parenthesis if not in one line (`#397 <https://github.com/ros2/system_tests/issues/397>`_)
* Remove ready_fn, and one self.proc_info (`#391 <https://github.com/ros2/system_tests/issues/391>`_)
* Clean up bounded sequences check (`#393 <https://github.com/ros2/system_tests/issues/393>`_)
  * Clean up bounded sequences check
  1. Use a macro to extract repeated logic
  2. Check the sequence length so we get a test error message instead of segfaulting.
  * add missing include
  * Check strings better
  Also, minor changes for clarity
  * rename EXPECT_RCSTR_EQ -> EXPECT_ROSIDLC_STREQ
* Contributors: Dan Rose, Dirk Thomas, Karsten Knese, Peter Baughman, William Woodall

0.8.0 (2019-11-20)
------------------
* 0.8.0
* Revert "Skip rclcpp__rclpy__rmw_connext_cpp__rmw_fastrtps_cpp tests (`#382 <https://github.com/ros2/system_tests/issues/382>`_)" (`#383 <https://github.com/ros2/system_tests/issues/383>`_)
  This reverts commit bde886a231ba262b2d8c1e81513b0c8f85e1f3bb.
* Skip rclcpp__rclpy__rmw_connext_cpp__rmw_fastrtps_cpp tests (`#382 <https://github.com/ros2/system_tests/issues/382>`_)
  These tests fail consistently due to an assertion in Fast-RTPS which was
  recently introduced.
* fix condition to not skip FastRTPS to FastRTPS pub/sub tests (`#377 <https://github.com/ros2/system_tests/issues/377>`_)
  * fix condition to not skip FastRTPS to FastRTPS pub/sub tests
  * fix and simplify comparison logic
  * skip WStrings on macOS from FastRTPS to Connext
* Support Arrays.srv in communication tests (`#376 <https://github.com/ros2/system_tests/issues/376>`_)
* Contributors: Dirk Thomas, Jacob Perron, Michael Carroll, Scott K Logan

0.7.1 (2019-05-29)
------------------

0.7.0 (2019-05-20)
------------------
* Fix memory leaks in test_communication tests (`#368 <https://github.com/ros2/system_tests/issues/368>`_)
  Fix memory leaks detected by AddressSanitizer in
  test_message_serialization and test_messages_c tests.
* Handle launch_testing assertExitCodes correctly (`#367 <https://github.com/ros2/system_tests/issues/367>`_)
* Fix deprecation warnings (`#364 <https://github.com/ros2/system_tests/issues/364>`_)
* Make test_subscriber_cpp always fail gracefully. (`#363 <https://github.com/ros2/system_tests/issues/363>`_)
  * Make test_subscriber_cpp always fail gracefully.
  * Use fprintf to stderr instead of std::cerr.
* changes to avoid deprecated API's (`#361 <https://github.com/ros2/system_tests/issues/361>`_)
  * changes to avoid deprecated API's
  * review comments
* Corrected publish calls with shared_ptr signature (`#348 <https://github.com/ros2/system_tests/issues/348>`_)
  * Corrected publish calls with shared_ptr signature
  * Updated with PR comments
  * Correct linter failure
* Fix issues with C messages test (`#355 <https://github.com/ros2/system_tests/issues/355>`_)
  * Initialize BasicTypes field of Array
  Otherwise, tests may fail if garbage values are used.
  * Correct the number of BoundedSequences messages
  * Add C message tests for Constants.msg, Defaults.msg, and Empty.msg
* skip cross vendor testing of OpenSplice for WStrings (`#354 <https://github.com/ros2/system_tests/issues/354>`_)
* update to be compatible with latest QoS changes (`#349 <https://github.com/ros2/system_tests/issues/349>`_)
* add WString tests (`#353 <https://github.com/ros2/system_tests/issues/353>`_)
* API updates for RMW preallocation work. (`#352 <https://github.com/ros2/system_tests/issues/352>`_)
* Use new interface definitions (`#350 <https://github.com/ros2/system_tests/issues/350>`_)
  * Replace Primitives with BasicTypes
  * Replace StaticArrayPrimitives with Arrays
  * Replace BoundedArrayPrimitives with BoundedSequences
  * Replace DynamicArrayPrimitives with UnboundedSequences
  * Replace nested sequence and array message types with MultiNested
  * Update FieldsWithSameType.msg
  * Update test_message_serialization.cpp
  * Fix compile issues
  * Add tests for Constants.msg and Defaults.msg
  * Update expected buffer length for serialized message
  * Add test for Strings.msg for C
  * cleanup
  * Update test_security package to use new interface definitions
  * Use BasicTypes for checking message serialization size
  It seems that BoundedSequences does not have a consistent size between tests.
* Migrate launch tests to new launch_testing features & API (`#340 <https://github.com/ros2/system_tests/issues/340>`_)
  * Update after launch_testing features becoming legacy.
  * Migrate test_rclcpp tests to new launch_testing API.
  * Migrate test_communication tests to new launch_testing API.
  * Migrate test_security tests to new launch_testing API.
  * Migrate test_cli_remapping tests to new launch_testing API.
  * Stop using injected attributes in launch tests.
  * Bump test_rclcpp tests timeout to please CI.
  * Fix PATH in test_security tests.
  * Bump test_security tests timeout to please CI.
  * Address peer review comments.
  * Please flake8 on test_cli_remapping.
* Update call to async_send_goal (`#346 <https://github.com/ros2/system_tests/issues/346>`_)
  * Update call to async_send_goal
  It now takes an options struct that contains a reference to the feedback callback.
  * Use action client method for requesting goal result
  Otherwise, an exception is thrown since there was no result callback provided when sending the goal.
* Rename action state transitions (`#342 <https://github.com/ros2/system_tests/issues/342>`_)
  * Rename action state transitions
  Now using active verbs as described in the design doc:
  http://design.ros2.org/articles/actions.html#goal-states
  Connects to `ros2/rcl#399 <https://github.com/ros2/rcl/issues/399>`_.
* Merge pull request `#339 <https://github.com/ros2/system_tests/issues/339>`_ from ros2/`ivanpauno/ros2#658 <https://github.com/ivanpauno/ros2/issues/658>`_
  Using ament_target_dependencies where possible
* Used ament_target_directories where possible in test_communication CMakeLists
* refactor test generation (`#336 <https://github.com/ros2/system_tests/issues/336>`_)
* update char type mapping, update to use separated action types (`#315 <https://github.com/ros2/system_tests/issues/315>`_)
  * update char type mapping
  * match renamed action types
  * use correct term
  * change char type values
  * make the build pass for now
  * update action API
  * update action API
* Add communication tests for Python Actions (`#333 <https://github.com/ros2/system_tests/issues/333>`_)
* Add launch along with launch_testing as test dependencies. (`#334 <https://github.com/ros2/system_tests/issues/334>`_)
* Drops legacy launch API usage. (`#328 <https://github.com/ros2/system_tests/issues/328>`_)
  * Drops legacy launch API usage.
  * Fixes style issues.
  * Drops more legacy launch API use cases.
  * Adds launch_testing as test_security dependency.
  * Applies misc fixes after Windows triaging.
  * Applies more fixes after Windows triaging.
  * Disables test_rclcpp cross vendor tests on Windows.
* Add test for test_msgs/NestedMessage.action (`#330 <https://github.com/ros2/system_tests/issues/330>`_)
  * Add test for test_msgs/NestedMessage.action
  * Fixes for cpplint and uncrustify
* pass context to wait set (`#324 <https://github.com/ros2/system_tests/issues/324>`_)
* Contributors: Dirk Thomas, Jacob Perron, M. M, Michael Carroll, Michel Hidalgo, Prajakta Gokhale, Shane Loretz, William Woodall, ivanpauno

0.6.0 (2018-12-14)
------------------
* Disable any cross-vendor communication tests for Fast-RTPS. (`#322 <https://github.com/ros2/system_tests/issues/322>`_)
  * Disable any cross-vendor communication tests for Fast-RTPS.
  Builds are actually failing for all cross-vendor tests involving
  rmw_fastrtps_cpp not just those between Connext and Fast-RTPS.
  * Address linter feedback.
* Disable cross vendor tests for pub/sub fastrtps/connext (`#320 <https://github.com/ros2/system_tests/issues/320>`_)
  * disable cross vendor tests for pub/sub fastrtps/connext
  * Only skip tests on Windows.
* add Fibonacci test for actions (`#316 <https://github.com/ros2/system_tests/issues/316>`_)
  * add Fibonacci test for actions
  * fixup test creation
  * remove debug code
  * action tests depend on action client and server
  * static cast to get rid of warning
* refactor to support init options and context (`#313 <https://github.com/ros2/system_tests/issues/313>`_)
  * refactor to support init options and context
  * fix security tests
  * pass context to timer api
  * avoid custom main just for init/shutdown
  * avoid terminate in ~thread on exceptions
  * update expected output
  * add missing fini in test fixture
  * fixup pub/sub test fixture
* only consider .msg files with a msg namespace (`#310 <https://github.com/ros2/system_tests/issues/310>`_)
* add new fixtures (`#312 <https://github.com/ros2/system_tests/issues/312>`_)
  * add new fixtures
  * fix copy paste error
* Update rcl_wait_set_add_guard_condition() call (`#311 <https://github.com/ros2/system_tests/issues/311>`_)
  Now the function takes an optional output index argument.
* Merge pull request `#307 <https://github.com/ros2/system_tests/issues/307>`_ from ros2/array-terminology
  rename dynamic array to sequence
* rename files
* rename dynamic array to sequence
* use new error handling API from rcutils (`#306 <https://github.com/ros2/system_tests/issues/306>`_)
  * use new error handling API from rcutils
  * fix some more cases where the new error handling API is used
* Merge pull request `#303 <https://github.com/ros2/system_tests/issues/303>`_ from ros2/hidmic/namespace-messages-with-subfolder
  Handles msg files with the same name in different subfolders
* Prevents tests from being generated for action messages and services.
* Handles msg files with the same name in different subfolders.
* Add new test message type DynamicArrayStaticArrayPrimitivesNested for communcation tests (`#302 <https://github.com/ros2/system_tests/issues/302>`_)
* use add_compile_options instead of setting only cxx flags
* Use consolidated rcl_wait_set_clear() (`#292 <https://github.com/ros2/system_tests/issues/292>`_)
* remove unused builtin_interfaces dependency (`#285 <https://github.com/ros2/system_tests/issues/285>`_)
* only test serialization on fastrtps and connext (`#284 <https://github.com/ros2/system_tests/issues/284>`_)
  * only test serialization on fastrtps and connext
  * use skip_test
  * reset skip test
* Expose cdr (`#267 <https://github.com/ros2/system_tests/issues/267>`_)
  * change to new rclcpp subscription api
  * uncrustify
  * add serialization tests
  * linters
  * add pub_sub test for raw callbacks
  * address review comments
  * warn unused
  * raw->serialized
  * use size_t (`#283 <https://github.com/ros2/system_tests/issues/283>`_)
  * raw->serialized
  * use size_t
* Use debug python executable on windows (`#281 <https://github.com/ros2/system_tests/issues/281>`_)
  * use debug python executable on windows
  * get python debug executable from pythonextra
* migrate launch -> launch.legacy (`#273 <https://github.com/ros2/system_tests/issues/273>`_)
* account for null-terminator character (`#269 <https://github.com/ros2/system_tests/issues/269>`_)
  * account for null-terminator character
  * modify tmpstr to be able to represent any size_t value
* Use call_async (`#257 <https://github.com/ros2/system_tests/issues/257>`_)
* [test_communication] Unique namespaces (`#256 <https://github.com/ros2/system_tests/issues/256>`_)
  * add namespace to pubsub tests
  * add namespace to service tests
  * uncrustify
  * use UTC time rather than datetime
  * Single quotes
  * make arguments mandatory like in C++
* Skip python service tests only for connext dynamic (`#249 <https://github.com/ros2/system_tests/issues/249>`_)
  * enable python services for all but connext dynamic
  * remove changes from 248
  * up to 20 spins
  * Revert "remove changes from 248"
  This reverts commit 77fc9f4b5e488533dfc8e079178ed17e2f8c288f.
* reenable service tests: rclcpp requester rclpy replier on Windows (`#248 <https://github.com/ros2/system_tests/issues/248>`_)
* Contributors: Alexis Pojomovsky, Dirk Thomas, Jacob Perron, Karsten Knese, Michel Hidalgo, Mikael Arguedas, Shane Loretz, Steven! Ragnarök, William Woodall

0.4.0 (2017-12-08)
------------------
* Update for rclcpp namespace removals (`#255 <https://github.com/ros2/system_tests/issues/255>`_)
  * Remove subscription:: namespace
  * Remove client:: namespace
  * Remove service:: namespace
  * Remove parameter_client:: namespace
  * Remove parameter_service:: namespace
  * Remove timer:: namespace
  * Remove node:: namespace
  * Remove event:: namespace
  * Remove utilities:: namespace
* Merge pull request `#252 <https://github.com/ros2/system_tests/issues/252>`_ from ros2/check_if_test_exists_before_adding_properties
  check if test exists before adding properties
* check if test exists before adding properties
* cmake 3.10 compatibility: pass absolute path to file(GENERATE) function (`#251 <https://github.com/ros2/system_tests/issues/251>`_)
* Wait for service before calling it (`#244 <https://github.com/ros2/system_tests/issues/244>`_)
  * Wait for service before calling it
  * Wait for a maximum of 15 seconds
  * Refactor to make sure cleanup happens
* find gtest before macro invocation so that its not find during each macro invocation (`#246 <https://github.com/ros2/system_tests/issues/246>`_)
* Merge pull request `#245 <https://github.com/ros2/system_tests/issues/245>`_ from ros2/ament_cmake_pytest
  use ament_cmake_pytest instead of ament_cmake_nose
* use ament_cmake_pytest instead of ament_cmake_nose
* typo
* Restore bigobj (`#241 <https://github.com/ros2/system_tests/issues/241>`_)
  * [test_communication] restore bigobj
  * [test_security] restore bigobj
  * make it explicit that bigobj is needed only in debug mode
* 240 fixups
* Replaces "std::cout<<" with "printf" (`#240 <https://github.com/ros2/system_tests/issues/240>`_)
  * [test_communication]replace uses of iostream
  * [test_rclcpp] remove use of std::cout except flushing
  * missed some
  * we use float duration not double
  * remove now unused include
* Merge pull request `#230 <https://github.com/ros2/system_tests/issues/230>`_ from ros2/test_connext_secure
  Test connext secure
* removing /bigobj flag on windows (`#239 <https://github.com/ros2/system_tests/issues/239>`_)
* move security tests in different package
  generate new security files with latest sros2 generation script
* Merge pull request `#236 <https://github.com/ros2/system_tests/issues/236>`_ from ros2/optimize_test_publisher_subscriber
  Minimize the number of calls to message.__repr_\_()
* Minimize the number of calls to message.__repr_\_()
* Merge pull request `#233 <https://github.com/ros2/system_tests/issues/233>`_ from ros2/uncrustify_master
  update style to match latest uncrustify
* n need to tweak python path now that messages come from test_msgs (`#232 <https://github.com/ros2/system_tests/issues/232>`_)
* update style to match latest uncrustify
* 0.0.3
* Test msgs (`#223 <https://github.com/ros2/system_tests/issues/223>`_)
  * use messages from test_msgs
  * update tests to use messages from new package
  * delete unused message files
  * update service tests as well
  * revert spurious changes
  * remove todo but dont change compile options because this package will keep generating it's own messages
  * no need to install isnterfaces anymore
  * rename message field for DynamicArrayPrimitivesNested
  * remove spurious line change
  * iterate over interface files to built list of services and messages
* Update test_messages_c.cpp (`#226 <https://github.com/ros2/system_tests/issues/226>`_)
  Array initialized with 2 while 3 elements filled, increased size.
* call rclcpp::shutdown in all tests (`#225 <https://github.com/ros2/system_tests/issues/225>`_)
* commenting out unused import for flake8 compliance
* Merge pull request `#222 <https://github.com/ros2/system_tests/issues/222>`_ from ros2/enable_array_tests_opensplice
  reenable array tests with OpenSplice
* reenable array tests with OpenSplice
* Ensure nodes have called rclcpp::shutdown before exiting (`#220 <https://github.com/ros2/system_tests/issues/220>`_)
* use unbuffered Python in launch files (`#218 <https://github.com/ros2/system_tests/issues/218>`_)
  * use unbuffered Python in launch files
  * use unbuffered Python in secure pubsub launch file
* testing array longers than 101 (`#216 <https://github.com/ros2/system_tests/issues/216>`_)
* Use _WIN32 everywhere (`#213 <https://github.com/ros2/system_tests/issues/213>`_)
* 0.0.2
* C memleak testing (`#211 <https://github.com/ros2/system_tests/issues/211>`_)
  * Added nested message that always breaks because of the bug
  * Added C++ code for DynamicArrayPrimitivesNested message
  * Fixed style and publisher/subscriber (combo) test case
  * expose core dumpes on complex messages
  * dont run other tests to save debugging time
  * more fixtures, looks like a string array alignment issue
  * newline at end of file
  * move include to the right place
  * add comment about current failing tests
  * remove debug prints
  * restore/reenable all tests
  * that was actually pretty readable with vertical space
  * use all messages fron the fixtures rather the only the first one
  * linters
  * what's cool with functions is that you can call them rather than copy-n-paste code
* destroy node before shutdown (`#210 <https://github.com/ros2/system_tests/issues/210>`_)
* use CMAKE_X_STANDARD and check compiler rather than platform
* add option for security tests (`#208 <https://github.com/ros2/system_tests/issues/208>`_)
* Adding security tests (`#204 <https://github.com/ros2/system_tests/issues/204>`_)
  * WIP: add security tests
  * keys, certs and crap used for testing
  * switching to a multi process test because of https://github.com/eProsima/Fast-RTPS/issues/106
  * test failing / throwing cases
  * test only for fastrtps for now
  * lint
  * unnused var name
  * WIP
  * test all message type for regression checking. Also disable should throw examples that will be implemented in a single process in C
  * update certs/key files
  * move tests with invalid node creation to single process
  * add not connecting tests with timer, remove unused args, simplify template logic
  * remove now useless topic_name parameters
  * leverage VALID_SECURE_ROOT
  * more cleanup
  * update copyright year
  * remove debug prints
  * remove unused variables
  * add generated from notice to all test python templates
  * removing variables is great, code that compiles is better
  * check for test target existence
  * rename test suite to match what is being tested
  * rename security environment variables
  * trailing whitespace
* destroy node before shutdown (`#207 <https://github.com/ros2/system_tests/issues/207>`_)
* Merge pull request `#205 <https://github.com/ros2/system_tests/issues/205>`_ from ros2/move_time
  remove unnecessary usage of RCL_S_TO_NS
* remove unnecessary usage of RCL_S_TO_NS
* remove unnecessary topic name check (`#203 <https://github.com/ros2/system_tests/issues/203>`_)
  * remove incorrect and unnecessary topic name check
  * up timeout for slow test
* set_tests_properties for correct requester replier executable (`#202 <https://github.com/ros2/system_tests/issues/202>`_)
* support addition of node namespace in rclcpp API (`#196 <https://github.com/ros2/system_tests/issues/196>`_)
* Merge pull request `#199 <https://github.com/ros2/system_tests/issues/199>`_ from ros2/use_explicit_kwargs
  use explicit kwargs
* use explicit kwargs
* Add missing exec dep on builtin_interfaces (`#198 <https://github.com/ros2/system_tests/issues/198>`_)
  * Add missing exec dep on builtin_interfaces
  * alphabetically is better
* Fix deps (`#192 <https://github.com/ros2/system_tests/issues/192>`_)
  * every day I'm reshuffling
  * auto
* Install msgs and fixtures for use by other packages (`#190 <https://github.com/ros2/system_tests/issues/190>`_)
  * Install msgs and fixtures for use by other packages
  * reshuffle depends
  * reshuffle depends
* Use -Wpedantic (`#189 <https://github.com/ros2/system_tests/issues/189>`_)
  * add pedantic flag
  * fix pedantic warning
  * fix C4456 warning
  * reduce scope of wait_sets
  * reduce scope rather than renaming variable
* Comply with flake8 + flake-import-order (`#188 <https://github.com/ros2/system_tests/issues/188>`_)
* Merge pull request `#187 <https://github.com/ros2/system_tests/issues/187>`_ from ros2/use_rmw_impl
  use rmw implementation
* remove usage of RCLPY_IMPLEMENTATION
* use rmw implementation
* Merge pull request `#186 <https://github.com/ros2/system_tests/issues/186>`_ from ros2/typesupport_c_reloaded
  use rosidl_typesupport_c
* use rosidl_typesupport_c
* replace deprecated <CONFIGURATION> with <CONFIG>
* use new rclcpp::literals namespace + constness issue fix (`#178 <https://github.com/ros2/system_tests/issues/178>`_)
  * use new rclcpp::literals namespace
  * test_subscription.cpp: fix missing 'const'
  wait_for_future() required a non-const reference but
  at the callers are using user-defined literals such as 10_s,
  which aren't lvalue.
  * add NOLINT to 'using namespace rclcpp::literals'
  * use std::chrono_literals
* c++14 (`#181 <https://github.com/ros2/system_tests/issues/181>`_)
* rclpy tests match rclcpp timing (`#183 <https://github.com/ros2/system_tests/issues/183>`_)
* Merge pull request `#180 <https://github.com/ros2/system_tests/issues/180>`_ from ros2/typesupport_reloaded
  append build space to library path
* test loong strings for services (`#179 <https://github.com/ros2/system_tests/issues/179>`_)
* append build space to library path
* Mark blacklisted tests as skipped (`#177 <https://github.com/ros2/system_tests/issues/177>`_)
  * skip opensplice failing tests
  * use new SKIP_TEST arg rather than hacking templates
  * lint cmake
  * remove unnecessary args
  * use _SKIP_TEST variable everywhere
  * rename _SKIP_TEST to SKIP_TEST
  * indent cmake
* Test python services (`#175 <https://github.com/ros2/system_tests/issues/175>`_)
  * extend service template to test python services
  * trailing whitespace
  * skipping tests raising SkipTest
  * remove SKIP_TEST for non nose tests
  * add bracket because linter doesnt understand multiline conditions
* remove unnecessary ament_index_build_path (`#174 <https://github.com/ros2/system_tests/issues/174>`_)
* use generator for target file location (`#173 <https://github.com/ros2/system_tests/issues/173>`_)
  * use generator for target file location
  * remove unused variable
* add a bunch of tests for rcl and rosidl_generator_c messages (`#122 <https://github.com/ros2/system_tests/issues/122>`_)
  * rcl tests for rosidl_generator_c and c type support
  * add test source file
  * don't need assignn
  * don't ignore fastrtps
  * test all message types
  * init messages with default values
  * increase test timeout
  * update fixtures
  * reuse primitive message verify function
  * no need for executables here
  * add waitset
  * increase string length
  * proper graph guard condition
* Merge pull request `#172 <https://github.com/ros2/system_tests/issues/172>`_ from ros2/fix_pyflakes
  fix pyflakes
* fix pyflakes
* Test cross RCL communication (`#152 <https://github.com/ros2/system_tests/issues/152>`_)
  * unify templates and configure them in a macro
  * remove unnecessary logic
  * reenable single process tests
  * refactor template parameters
  * reenable service testing across rmw
  * string compare
  * wrap blacklist tests condition
  * clean comments
  * reenable failing connext_dynamic StaticArrayNested test
  * remove env variable check
  * rename rcl variable to client_library(ies)
  * rename macro
* Merge pull request `#171 <https://github.com/ros2/system_tests/issues/171>`_ from ros2/rosidl_target_interfaces_add_dependency
  remove obsolete add_dependencies
* remove obsolete add_dependencies
* support local graph changes in Connext (`#164 <https://github.com/ros2/system_tests/issues/164>`_)
  * remove blocks and workarounds on service tests
  * remove no longer needed sleep
  * remove blocks and workarounds on new service test
  * replace busy wait with graph event wait
  * use new non-busy wait
  * [style] uncrustify and cpplint
  * increase timeout for test_services
  timeout was 30s, but it is consistently taking
  34s for me
  * update wait_for_subscriber to also wait for it to be gone
  * deduplicate code and allow retried publishing
  * increase timeout for test_rclcpp/test_subscription to 60s
  * comment cleanup
  * Fix typo
* Fixed tests after pull request `ros2/rclcpp#261 <https://github.com/ros2/rclcpp/issues/261>`_ (`#170 <https://github.com/ros2/system_tests/issues/170>`_)
* Merge pull request `#168 <https://github.com/ros2/system_tests/issues/168>`_ from ros2/looong_strings
  tests strings > 256
* tests strings > 256
* Merge pull request `#166 <https://github.com/ros2/system_tests/issues/166>`_ from ros2/fix_cpplint
  comply with stricter cpplint rules
* comply with stricter cpplint rules
* increase max spin count to handle fastrtps different spin behaviour
* enable fastrtps python tests
* Ascii fixture (`#161 <https://github.com/ros2/system_tests/issues/161>`_)
  * use ASCII value for char
  * homogenize BoundedArrayPrimitives fixtures
* Merge pull request `#148 <https://github.com/ros2/system_tests/issues/148>`_ from ros2/remove_noop
  remove noops
* remove noops
* Merge pull request `#147 <https://github.com/ros2/system_tests/issues/147>`_ from ros2/fix_more_tests
  remove duplicates of test_subscription_valid_data_cpp, fix skipped tests on Windows
* remove duplicates of test_subscription_valid_data_cpp, fix skipped tests on Windows
* Merge pull request `#146 <https://github.com/ros2/system_tests/issues/146>`_ from ros2/revert_test_requester_timing
  revert test requester timing
* revert test requester timing
* update schema url
* Merge pull request `#145 <https://github.com/ros2/system_tests/issues/145>`_ from ros2/sleep_if_not_wait_for_service
  use sleep if wait_for_service throws
* use sleep if wait_for_service throws
* add schema to manifest files
* Merge pull request `#142 <https://github.com/ros2/system_tests/issues/142>`_ from ros2/bounded_vector
  add tests for bounded vectors
* Merge pull request `#144 <https://github.com/ros2/system_tests/issues/144>`_ from ros2/update_test_times
  update test times
* support bounded vectors
* add communication tests for bounded arrays
* update test times
* Use wait_for_service to make Service tests less flaky (`#132 <https://github.com/ros2/system_tests/issues/132>`_)
  * use wait_for_service to make tests less flaky
  * realign timeouts
  * avoid using wait_for_service with fastrtps
  this can be undone once fastrtps supports wait_for_service
  * [test_communication] avoid wait_for_service with fastrtps
  it can be undone once fastrtps supports wait_for_service
  * add test to ensure wait_for_service wakes after shutdown/sigint
* Windows python debug (`#138 <https://github.com/ros2/system_tests/issues/138>`_)
  * pass python interpreter to nose test
  * rename interpreter to executable
  * rename PYTHON_DBG_EXECUTABLE to PYTHON_EXECUTABLE_DEBUG
  * cmake3.5 remove variable expansion
* add tests for all message_files (`#125 <https://github.com/ros2/system_tests/issues/125>`_)
  * add tests for all messages
  * do not run opensplice failing test
  * use cmake3.5 syntax, fixed rmw_implementation variable
* dont assert type support during import (`#141 <https://github.com/ros2/system_tests/issues/141>`_)
* wrap complex condition
* Merge pull request `#136 <https://github.com/ros2/system_tests/issues/136>`_ from ros2/cmake35
  require CMake 3.5
* remove trailing spaces from comparisons, obsolete quotes and explicit variable expansion
* require CMake 3.5
* linting
* fix string comparison cmake
* Merge pull request `#121 <https://github.com/ros2/system_tests/issues/121>`_ from ros2/add_rclpy_talker_listener_to_test_communication
  add tests for rclpy talker listener
* check the rmw id matches in cross-vendor tests (`#126 <https://github.com/ros2/system_tests/issues/126>`_)
* rclpy from install folder
* revert cpp tests addition, handled by `#125 <https://github.com/ros2/system_tests/issues/125>`_
* cleanup
* added todo for rmw blacklist in cmake
* reenable cpp tests
* use camelcase format for message names
* already enforced by argparse
* move testing in callback to match cpp tests
* extend default duration to allow missed messages
* removed artefacts from poor rebase, blacklist failing opensplice dynamicarrayprimitives
* disable non python tests for testing on the farm
* test python for all message types
* add libs for windows
* fix assert condition and pep8
* use utf-8 compatible values for char testing
* enable test for all msg files
* testing values cross rcl communication
* multiple rmw_implementation
* macro
* use fixture, test received message
* add python message fixtures
* add comment for system path inserts
* add rclpy directory to system path
* add rclpy as test dependency
* homogenize fiels assignment for testing
* fixed byte/char array assignation
* added back char and byte now that somehow fixed on python generator side
  Conflicts:
  test_communication/test/message_fixtures.hpp
* working towards testing python communication along cpp one
  Conflicts:
  test_communication/test/message_fixtures.hpp
* Merge pull request `#119 <https://github.com/ros2/system_tests/issues/119>`_ from ros2/fix_tests
  fix generation of tests with multiple executables
* disable cross vendor services for FastRTPS
* disable tests failing due to OpenSplice bug
* Merge pull request `#128 <https://github.com/ros2/system_tests/issues/128>`_ from ros2/curly_brackets
  fix use of brackets
* fix brackets: see if Clang or Windows complains
* remove RCL_ASSERT_RMW_ID_MATCHES for multi target tests
* fix generation of tests with multiple executables
* Merge pull request `#127 <https://github.com/ros2/system_tests/issues/127>`_ from ros2/float_fixture_values
  use values that fit in a float for testing
* use values that fit in a float for testing
* fix spelling in comment
* Merge pull request `#120 <https://github.com/ros2/system_tests/issues/120>`_ from dhood/test-linking-runtime
  Ensure using correct rmw implementation in tests
* Use RCL_ASSERT_RMW_ID_MATCHES to ensure correct rmw implementation is being used
* move message registration
* Merge pull request `#118 <https://github.com/ros2/system_tests/issues/118>`_ from ros2/rclcpp219
  extend test to cover another case
* Merge pull request `#117 <https://github.com/ros2/system_tests/issues/117>`_ from ros2/msg_with_fields_with_same_type
  add message which has fields with the same non-primitive type
* extend test to cover `ros2/rclcpp#219 <https://github.com/ros2/rclcpp/issues/219>`_
* add message which has fields with the same non-primitive type
* Merge pull request `#115 <https://github.com/ros2/system_tests/issues/115>`_ from ros2/ctest_build_testing
  use CTest BUILD_TESTING
* Get only C++ typesupport implementations (`#114 <https://github.com/ros2/system_tests/issues/114>`_)
  * Get only C++ typesupport implementations
  * Add busy_wait_for_subscriber to make publisher test unflaky
* use CTest BUILD_TESTING
* Use rcl (`#113 <https://github.com/ros2/system_tests/issues/113>`_)
  * init is required now
  * Fix multiple init calls
  * Add init to a test, increase timeout and change an assertion to an expectation
  * Fix argc/argv
  * wait for subscriber in publisher test
* Merge pull request `#105 <https://github.com/ros2/system_tests/issues/105>`_ from ros2/generator_expression
  use generator expressions for configuration specific tests
* use generator expressions for configuration specific tests
* Merge pull request `#102 <https://github.com/ros2/system_tests/issues/102>`_ from ros2/rename_message_type_support
  support multiple type supports per rmw impl
* support multiple type supports per rmw impl
* Merge pull request `#101 <https://github.com/ros2/system_tests/issues/101>`_ from ros2/windows_release
  build release on Windows
* build release on Windows
* Merge pull request `#86 <https://github.com/ros2/system_tests/issues/86>`_ from ros2/refactor_typesupport
  use new approach to generate rmw implementation specific targets
* use new approach to generate rmw implementation specific targets
* Merge pull request `#83 <https://github.com/ros2/system_tests/issues/83>`_ from ros2/missing_dep
  add missing dependency on rmw_implementation_cmake
* add missing dependency on rmw_implementation_cmake
* Merge pull request `#59 <https://github.com/ros2/system_tests/issues/59>`_ from ros2/cpplint
  update code to pass ament_cpplint
* update code to pass ament_cpplint
* Merge pull request `#58 <https://github.com/ros2/system_tests/issues/58>`_ from ros2/optional-qos-profile
  Made rmw_qos_profile argument optional
* Made rmw_qos_profile argument optional
* Merge pull request `#42 <https://github.com/ros2/system_tests/issues/42>`_ from ros2/test-services
  Added tests for services
* Added tests for services
* Merge pull request `#51 <https://github.com/ros2/system_tests/issues/51>`_ from ros2/issue_50
  disregard duplicate requests
* Merge pull request `#52 <https://github.com/ros2/system_tests/issues/52>`_ from ros2/reduce_test_times
  Reduce test times
* remove all references to received_messages
* rename rate variables
* reduce test times
* make duplicate requests just a warning not a failure
  fixes `#50 <https://github.com/ros2/system_tests/issues/50>`_
* Merge pull request `#34 <https://github.com/ros2/system_tests/issues/34>`_ from ros2/wrong_service_callback
  update test to catch repeated service callbacks
* Merge pull request `#35 <https://github.com/ros2/system_tests/issues/35>`_ from ros2/rmw_gid_support
  update intra proc tests with different assumptions
* update intra proc tests with different assumptions
* update test to catch repeated service callbacks
* Merge pull request `#17 <https://github.com/ros2/system_tests/issues/17>`_ from ros2/unbounded
  add fixtures with longer dynamic content
* add fixtures with a string with more then 255 characted and more than 100 elements in a sequence
* Merge pull request `#27 <https://github.com/ros2/system_tests/issues/27>`_ from ros2/check_sample_valid_data
  add test to check for receiving callbacks for invalid data
* add test to check for receiving callbacks for invalid data
* Merge pull request `#24 <https://github.com/ros2/system_tests/issues/24>`_ from ros2/qos
  Added support for QoS profiles
* Added support for QoS profiles
* use linters
* [style] limit line length to 100 chars.
* Merge pull request `#21 <https://github.com/ros2/system_tests/issues/21>`_ from ros2/fix_more_windows_warnings
  fix more windows warnings
* fix more windows warnings
* add explicit build type
* Merge pull request `#18 <https://github.com/ros2/system_tests/issues/18>`_ from ros2/raise_warning_level
  raise warning level
* raise warning level
* Merge pull request `#15 <https://github.com/ros2/system_tests/issues/15>`_ from ros2/test_array_submsgs
  add test to cover messages with an array of sub messages
* add test to cover messages with a static array of sub messages
* improve error messages
* remove package name prefix
* add test to cover messages with an array of sub messages
* fix generation of test results for successful tests
* Merge pull request `#13 <https://github.com/ros2/system_tests/issues/13>`_ from ros2/single_process_pub_sub
  add tests for publish/subscribe in a single process
* add tests for publish/subscribe in a single process
* Merge pull request `#12 <https://github.com/ros2/system_tests/issues/12>`_ from ros2/refactor_examples_and_interfaces
  changes to support renaming of interface packages
* changes to support renaming of interface packages
* Merge pull request `#11 <https://github.com/ros2/system_tests/issues/11>`_ from ros2/update_message_api
  update message API
* update message API
* Merge pull request `#6 <https://github.com/ros2/system_tests/issues/6>`_ from ros2/wjwwood_warnings_cleanup
  adjust use of braces to fix warnings with clang
* adjust use of braces to fix warnings with clang
* Merge pull request `#9 <https://github.com/ros2/system_tests/issues/9>`_ from ros2/fix_narrowing_conversion_error_windows
  fix narrowing conversion error on windows
* fix narrowing conversion error on windows
* Merge pull request `#5 <https://github.com/ros2/system_tests/issues/5>`_ from ros2/refactor_msg_gen
  refactor message generation
* refactor message generation (`ros2/ros2#48 <https://github.com/ros2/ros2/issues/48>`_)
* Merge pull request `#4 <https://github.com/ros2/system_tests/issues/4>`_ from ros2/test_nested
  add tests for nested messages
* add tests for builtin messages
* add tests for nested messages
* Merge pull request `#3 <https://github.com/ros2/system_tests/issues/3>`_ from ros2/dynamic_arrays
  add test for messages with dynamic arrays
* use double curly braces on vector init lists to work on Windows
* add test for messages with dynamic arrays
* Merge pull request `#2 <https://github.com/ros2/system_tests/issues/2>`_ from ros2/static_arrays
  add test for messages with static arrays
* add test for messages with static arrays
* remove obsolete comments
* Merge pull request `#1 <https://github.com/ros2/system_tests/issues/1>`_ from ros2/first_tests
  add generic tests for pub/sub and req/rep, add two message and service types for now
* add generic tests for pub/sub and req/rep, add two message and service types for now
* Contributors: Dirk Thomas, Esteve Fernandez, Guillaume Papin, Jackie Kay, Mikael Arguedas, Morgan Quigley, Rafał Kozik, Shane Loretz, William Woodall, dhood, gerkey, michielb
