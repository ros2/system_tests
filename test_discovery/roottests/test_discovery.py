# Run discovery tests
# 1. Install dependencies
#   sudo apt install iputils-ping iproute2 mininet
#    TODO(sloretz) not needed? openvswitch-switch openvswitch-testcontroller
# 2. TODO(sloretz) not needed? Start openvswitch service if not already running
#   sudo service openvswitch-switch start
# 3. Run this file as root
#   cd path/to/dir/containing/this/file/
#   sudo python3 -m pytest -c conftest.py --ros-workspace ~/ws/ros2/install ./test_all.py
# 3a. Use -k to limit which rmw implementation
#   sudo python3 -m pytest -k rmw_fastrtps_cpp -c conftest.py --ros-workspace ~/ws/ros2/install
# 3b. Running one test with all output
#   sudo python3 -m pytest -vv -k 'test_samehost[no_peer-LOCALHOST-no_peer-LOCALHOST-rmw_fastrtps_cpp]' -c conftest.py --ros-workspace ~/ws/ros2/install ./test_all.py

import pytest
from mininet.net import Mininet
from mininet.util import dumpNetConnections
from mininet.topo import MinimalTopo


RANGES = [
    "OFF",
    "SUBNET",
    "LOCALHOST",
]


class MininetFixture:
    __slots__ = (
        'net',
        'h1',
        'h2',
    )


def h1_ipv4(net: MininetFixture) -> str:
    return net.h1.IP()


def h2_ipv4(net: MininetFixture) -> str:
    return net.h2.IP()


def no_peer(net: MininetFixture) -> str:
    return ""


@pytest.fixture()
def mn():
    f = MininetFixture()
    f.net = Mininet(topo=MinimalTopo())
    f.h1 = f.net.getNodeByName('h1')
    f.h2 = f.net.getNodeByName('h2')

    f.net.start()
    yield f
    f.net.stop()


# TODO(sloretz) figure out ROS workspace path from environment variables
@pytest.fixture(scope="session")
def ros_ws(pytestconfig):
    return pytestconfig.getoption("ros_workspaces").split(':')


def make_env_str(ros_ws, rmw, range, peer):
    cmd = []
    for ws in ros_ws:
        cmd.append('.')
        cmd.append(f'"{ws}/setup.bash"')
        cmd.append('&&')
    cmd.append(f'RMW_IMPLEMENTATION={rmw}')
    cmd.append(f'ROS_AUTOMATIC_DISCOVERY_RANGE={range}')
    cmd.append(f'ROS_STATIC_PEERS="{peer}"')
    cmd.append(' ')
    return ' '.join(cmd)

# TODO samehost doesn't require mininet does it. Could do this with normal testing
@pytest.mark.parametrize("sub_peer", (no_peer, h1_ipv4))
@pytest.mark.parametrize("sub_range", RANGES)
@pytest.mark.parametrize("pub_peer", (no_peer, h1_ipv4))
@pytest.mark.parametrize("pub_range", RANGES)
def test_samehost(mn, ros_ws, rmw, pub_range, pub_peer, sub_range, sub_peer):
    pub_peer = pub_peer(mn)
    sub_peer = sub_peer(mn)

    pub_cmd = make_env_str(ros_ws, rmw, pub_range, pub_peer) + 'ros2 run test_discovery publish_once > /dev/null &'
    sub_cmd = make_env_str(ros_ws, rmw, sub_range, sub_peer) + 'ros2 run test_discovery subscribe_once'
    print("$", pub_cmd)
    print("$", sub_cmd)

    mn.h1.cmd(pub_cmd)
    result = mn.h1.cmd(sub_cmd)
    message_received = "test_discovery: message was received" in result.strip()

    if pub_peer or sub_peer:
        # if either has a static peer set, discovery should succeed
        assert message_received, result.strip()
    elif "OFF" in (pub_range, sub_range):
        # With no static peer, if either has discovery off then it won't succeed
        assert not message_received, result.strip()
    else:
        # All other cases discovery
        assert message_received, result.strip()


@pytest.mark.parametrize("sub_peer", (no_peer, h1_ipv4))
@pytest.mark.parametrize("sub_range", RANGES)
@pytest.mark.parametrize("pub_peer", (no_peer, h2_ipv4))
@pytest.mark.parametrize("pub_range", RANGES)
def test_differenthost(mn, ros_ws, rmw, pub_range, pub_peer, sub_range, sub_peer):
    pub_peer = pub_peer(mn)
    sub_peer = sub_peer(mn)

    pub_cmd = make_env_str(ros_ws, rmw, pub_range, pub_peer) + 'ros2 run test_discovery publish_once > /dev/null &'
    sub_cmd = make_env_str(ros_ws, rmw, sub_range, sub_peer) + 'ros2 run test_discovery subscribe_once'
    print("$", pub_cmd)
    print("$", sub_cmd)

    mn.h1.cmd(pub_cmd)
    result = mn.h2.cmd(sub_cmd)
    message_received = "test_discovery: message was received" in result.strip()

    if pub_peer or sub_peer:
        # if either has a static peer set, discovery should succeed
        assert message_received, result.strip()
    elif "SUBNET" == pub_range and "SUBNET" == sub_range:
        # With no static peer, succeed only if both are set to SUBNET
        assert message_received, result.strip()
    else:
        # All other cases discovery
        assert not message_received, result.strip()
