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

import os
import pathlib
import subprocess
import sys

from ament_index_python import get_package_prefix
from ament_index_python import get_resources
import pytest


RANGES = [
    # None,  # idential to LOCALHOST, but takes too long to test here
    'OFF',
    'SUBNET',
    'LOCALHOST',
]


PEERS = [
    None,
    '127.0.0.1',
]


def get_rmw_implementations():
    resources = list(get_resources('rmw_typesupport').keys())
    if 'rmw_implementation' in resources:
        resources.remove('rmw_implementation')
    return tuple(resources)


def get_executable(name):
    p = pathlib.Path(get_package_prefix('test_discovery'))
    return str(p / 'lib' / 'test_discovery' / name)


def make_env(rmw, discovery_range, peer):
    env = dict(os.environ)
    env['RMW_IMPLEMENTATION'] = rmw
    if discovery_range is None:
        del env['ROS_AUTOMATIC_DISCOVERY_RANGE']
    else:
        env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = discovery_range
    if peer is None:
        peer = ''
    env['ROS_STATIC_PEERS'] = peer
    return env


def communicate(name, proc):
    try:
        stdout, stderr = proc.communicate(timeout=10)
    except subprocess.TimeoutExpired:
        proc.kill()
        stdout, stderr = proc.communicate()
    stdout = stdout.decode()
    stderr = stderr.decode()
    for line in stdout.split('\n'):
        sys.stdout.write(f'{name}[stdout]: {line}\n')
    for line in stderr.split('\n'):
        sys.stderr.write(f'{name}[stderr]: {line}\n')
    return stdout, stderr


@pytest.mark.parametrize('sub_peer', PEERS)
@pytest.mark.parametrize('sub_range', RANGES)
@pytest.mark.parametrize('pub_peer', PEERS)
@pytest.mark.parametrize('pub_range', RANGES)
@pytest.mark.parametrize('rmw', get_rmw_implementations())
def test_thishost(rmw, pub_range, pub_peer, sub_range, sub_peer):
    # For same host tests, setting a static peer while using SUBNET
    # doesn't make a lot of sense.
    # Further, it cause test failures with rmw_fastrtps_*
    # When there's an initial peer to localhost, Fast-DDS will try to discover
    # peers on localhost using unicast discovery, but that uses the
    # default maxInitialPeersRange value of 4, which is a very small number of
    # peers.
    # Any other ROS nodes on a system (such as daemons, or test processes that
    # weren't cleaned up) will cause this test to fail.
    if 'SUBNET' == sub_range and sub_peer is not None:
        pytest.skip('Skipping samehost SUBNET with static peer')
    if 'SUBNET' == pub_range and pub_peer is not None:
        pytest.skip('Skipping samehost SUBNET with static peer')

    pub_env = make_env(rmw, pub_range, pub_peer)
    sub_env = make_env(rmw, sub_range, sub_peer)
    pub_cmd = [get_executable('publish_once')]
    sub_cmd = [get_executable('subscribe_once')]

    pub_proc = subprocess.Popen(
        pub_cmd, env=pub_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    sub_proc = subprocess.Popen(
        sub_cmd, env=sub_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    stdout, _ = communicate('sub', sub_proc)
    # Invalid node configuration could make OFF tests appear to succeed
    assert 'test_discovery: node successfully created' in stdout

    message_received = 'test_discovery: message was received' in stdout
    pub_proc.kill()

    communicate('pub', pub_proc)

    if 'OFF' in (pub_range, sub_range):
        # If either has discovery off then it won't succeed
        assert not message_received
    else:
        # All other cases discover each other
        assert message_received
