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
    "OFF",
    "SUBNET",
    "LOCALHOST",
]


PEERS = [
    None,
    "127.0.0.1",
]

def get_rmw_implementations():
    resources = list(get_resources('rmw_typesupport').keys())
    if 'rmw_implementation' in resources:
        resources.remove('rmw_implementation')
    return tuple(resources)[:1]


def get_executable(name):
    p = pathlib.Path(get_package_prefix('test_discovery'))
    return str(p / 'lib' / 'test_discovery' / name)


def make_env(rmw, range, peer):
    env = dict(os.environ)
    env['RMW_IMPLEMENTATION'] = rmw
    env['ROS_AUTOMATIC_DISCOVERY_RANGE'] = range
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


@pytest.mark.parametrize("sub_peer", PEERS)
@pytest.mark.parametrize("sub_range", RANGES)
@pytest.mark.parametrize("pub_peer", PEERS)
@pytest.mark.parametrize("pub_range", RANGES)
@pytest.mark.parametrize("rmw", get_rmw_implementations())
def test_thishost(rmw, pub_range, pub_peer, sub_range, sub_peer):
    pub_env = make_env(rmw, pub_range, pub_peer)
    sub_env = make_env(rmw, sub_range, sub_peer)
    pub_cmd = [get_executable('publish_once')]
    sub_cmd = [get_executable('subscribe_once')]

    pub_proc = subprocess.Popen(
        pub_cmd, env=pub_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    sub_proc = subprocess.Popen(
        sub_cmd, env=sub_env, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    stdout, _ = communicate('sub', sub_proc)
    message_received = "test_discovery: message was received" in stdout
    pub_proc.kill()
    communicate('pub', pub_proc)

    if pub_peer or sub_peer:
        # if either has a static peer set, discovery should succeed
        assert message_received
    elif "OFF" in (pub_range, sub_range):
        # With no static peer, if either has discovery off then it won't succeed
        assert not message_received
    else:
        # All other cases discovery
        assert message_received
