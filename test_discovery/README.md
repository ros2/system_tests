# test_discovery

This package tests the use of `ROS_AUTOMATIC_DISCOVERY_RANGE` and `ROS_STATIC_PEERS` environment variables.
There are two sets of tests: automated tests that run when testing this package, and semiautomated tests that must be run manually after building this package.

# Automated tests

The automated tests run when testing this package.
They test only the cases taht apply when two processes are on the same host.

The expected commication with two processes on the same host is:

|Same host |||Node B setting ||||||
|---|---|---|---|---|---|---|---|---|
||||No static peer |||With static peer |||
|||Range |OFF |LOCALHOST |SUBNET |OFF |LOCALHOST |SUBNET|
|Node A setting |No static peer |OFF |:x: |:x: |:x: |:x: |:x: |:x:|
|||LOCALHOST |:x: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
|||SUBNET |:x: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
||With static peer |OFF |:x: |:x: |:x: |:x: |:x: |:x:|
|||LOCALHOST |:x: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
|||SUBNET |:x: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|

## Semi-automated tests

The semiautomated tests use `mininet` to test discovery behavior across two different (virtual) hosts.
These tests require `root` access.

The expected communication with two processes on different hosts is:

|Different hosts |||Node B setting ||||||
||||No static peer |||With static peer |||
|||Range |OFF |LOCALHOST |SUBNET |OFF |LOCALHOST |SUBNET|
|Node A setting |No static peer |OFF |:x: |:x: |:x: |:x: |:x: |:x:|
|||LOCALHOST |:x: |:x: |:x: |:x: |:white_check_mark: |:white_check_mark:|
|||SUBNET |:x: |:x: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
||With static peer |OFF |:x: |:x: |:x: |:x: |:x: |:x:|
|||LOCALHOST |:white_check_mark: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
|||SUBNET |:white_check_mark: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
|||LOCALHOST |:x: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|
|||SUBNET |:x: |:white_check_mark: |:white_check_mark: |:x: |:white_check_mark: |:white_check_mark:|


### Installing prerequisites

### Running the semi-automated tests.
