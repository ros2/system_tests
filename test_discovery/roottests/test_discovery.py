# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Run discovery tests
# 1. Install dependencies
#   sudo apt install iputils-ping iproute2 mininet openvswitch-switch openvswitch-testcontroller
# 2. Start openvswitch service if not already running
#   sudo service openvswitch-switch start

from mininet.net import Mininet
from mininet.topo import MinimalTopo
import pytest


RANGES = [
    # None,  # idential to LOCALHOST, but takes too long to test here
    'OFF',
    'SUBNET',
    'LOCALHOST',
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
    return ''


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
@pytest.fixture(scope='session')
def ros_ws(pytestconfig):
    return pytestconfig.getoption('ros_workspaces').split(':')


def make_env_str(ros_ws, rmw, discovery_range, peer):
    cmd = []
    for ws in ros_ws:
        cmd.extend([
            '.',
            f'"{ws}/setup.bash"',
            '&&',
        ])
    cmd.extend([
        f'RMW_IMPLEMENTATION={rmw}',
        f'ROS_STATIC_PEERS="{peer}"',
    ])
    if discovery_range is not None:
        cmd.append(f'ROS_AUTOMATIC_DISCOVERY_RANGE={discovery_range}')
    return ' '.join(cmd)


@pytest.mark.parametrize('sub_peer', (no_peer, h1_ipv4))
@pytest.mark.parametrize('sub_range', RANGES)
@pytest.mark.parametrize('pub_peer', (no_peer, h2_ipv4))
@pytest.mark.parametrize('pub_range', RANGES)
def test_differenthost(mn, ros_ws, rmw, pub_range, pub_peer, sub_range, sub_peer):
    pub_peer = pub_peer(mn)
    sub_peer = sub_peer(mn)

    pub_env = make_env_str(ros_ws, rmw, pub_range, pub_peer)
    sub_env = make_env_str(ros_ws, rmw, sub_range, sub_peer)
    pub_cmd = pub_env + ' ros2 run test_discovery publish_once > /dev/null &'
    sub_cmd = sub_env + ' ros2 run test_discovery subscribe_once'
    print('$', pub_cmd)
    print('$', sub_cmd)

    mn.h1.cmd(pub_cmd)
    result = mn.h2.cmd(sub_cmd)

    # Invalid node configuration could make OFF tests appear to succeed
    assert 'test_discovery: node successfully created' in result.strip()

    message_received = 'test_discovery: message was received' in result.strip()

    if 'OFF' in (pub_range, sub_range):
        # If either has discovery off then it won't succeed
        assert not message_received, result.strip()
    elif pub_peer or sub_peer:
        # if either has a static peer set, discovery should succeed
        assert message_received, result.strip()
    elif 'SUBNET' == pub_range and 'SUBNET' == sub_range:
        # With no static peer, succeed only if both are set to SUBNET
        assert message_received, result.strip()
    else:
        # All other cases don't discover each other
        assert not message_received, result.strip()
