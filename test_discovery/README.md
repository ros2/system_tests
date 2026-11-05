# test_discovery

This package tests the use of `ROS_AUTOMATIC_DISCOVERY_RANGE` and `ROS_STATIC_PEERS` environment variables.
There are two sets of tests: automated tests that run when testing this package, and semiautomated tests that must be run manually after building this package.

# Automated tests

The automated tests run when testing this package.
They test only the cases that apply when two processes are on the same host.

## Semi-automated tests

The semiautomated tests use `mininet` to test discovery behavior across two different (virtual) hosts.
These tests require `root` access, and a working `mininet` install.

### Installing prerequisites

A working `mininet` install has only been tested on an Ubuntu based machine.
If you're running in a container, that container will need root priviledges.

First install the necessary dependencies:

```bash
sudo apt install \
    iputils-ping \
    iproute2 \
    mininet \
    openvswitch-switch \
    openvswitch-testcontroller
```

Next, make sure the openvswitch service is running

```bash
sudo service openvswitch-switch start
```

You're now ready to run the tests.

### Running the semi-automated tests
