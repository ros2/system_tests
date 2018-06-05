# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import asyncio
import functools
import os
import random
import sys
import time

from launch.legacy import LaunchDescriptor
from launch.legacy.exit_handler import primary_exit_handler
from launch.legacy.launcher import DefaultLauncher
import pytest
import rclpy


def get_environment_variable(name):
    """Get environment variable or raise if it does not exist."""
    path = os.getenv(name)
    if not path:
        raise EnvironmentError('Missing environment variable "%s"' % name)
    return path


CLIENT_LIBRARY_EXECUTABLES = (
    get_environment_variable('NAME_MAKER_RCLCPP'),
    get_environment_variable('NAME_MAKER_RCLPY')
)


@pytest.fixture(scope='module', params=CLIENT_LIBRARY_EXECUTABLES)
def node_fixture(request):
    """Create a fixture with a node, name_maker executable, and random string."""
    rclpy.init()
    node = rclpy.create_node('test_cli_remapping')
    try:
        yield {
            'node': node,
            'executable': request.param,
            'random_string': '%d_%s' % (
                random.randint(0, 9999), time.strftime('%H_%M_%S', time.gmtime()))
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def remapping_test(*, cli_args):
    """Return a decorator that returns a test function."""
    def real_decorator(coroutine_test):
        """Return a test function that runs a coroutine test in a loop with a launched process."""
        nonlocal cli_args

        @functools.wraps(coroutine_test)
        def test_func(node_fixture):
            """Run an executable with cli_args and coroutine test in the same asyncio loop."""
            nonlocal cli_args

            # Create a command launching a name_maker executable specified by the pytest fixture
            command = [node_fixture['executable']]
            # format command line arguments with random string from test fixture
            for arg in cli_args:
                command.append(arg.format(random_string=node_fixture['random_string']))

            # Execute python files using same python used to start this test
            env = dict(os.environ)
            if command[0][-3:] == '.py':
                command.insert(0, sys.executable)
                env['PYTHONUNBUFFERED'] = '1'

            ld = LaunchDescriptor()
            ld.add_process(
                cmd=command,
                name='name_maker_' + coroutine_test.__name__,
                env=env
            )
            ld.add_coroutine(
                coroutine_test(node_fixture),
                name=coroutine_test.__name__,
                exit_handler=primary_exit_handler
            )
            launcher = DefaultLauncher()
            launcher.add_launch_descriptor(ld)
            return_code = launcher.launch()
            assert return_code == 0, 'Launch failed with exit code %r' % (return_code,)
        return test_func
    return real_decorator


def get_topics(node_fixture):
    topic_names_and_types = node_fixture['node'].get_topic_names_and_types()
    return [name for name, _ in topic_names_and_types]


def get_services(node_fixture):
    service_names_and_types = node_fixture['node'].get_service_names_and_types()
    return [name for name, _ in service_names_and_types]


ATTEMPTS = 10
TIME_BETWEEN_ATTEMPTS = 1


@remapping_test(cli_args=('__node:=node_{random_string}',))
async def test_node_name_replacement_new(node_fixture):
    node_name = 'node_{random_string}'.format(**node_fixture)

    for attempt in range(ATTEMPTS):
        if node_name in node_fixture['node'].get_node_names():
            break
        await asyncio.sleep(TIME_BETWEEN_ATTEMPTS)
        rclpy.spin_once(node_fixture['node'], timeout_sec=0)
    assert node_name in node_fixture['node'].get_node_names()


@remapping_test(cli_args=('__ns:=/ns/s{random_string}',))
async def test_namespace_replacement(node_fixture):
    name = '/ns/s{random_string}/relative/name'.format(**node_fixture)

    for attempt in range(ATTEMPTS):
        if name in get_topics(node_fixture) and name in get_services(node_fixture):
            break
        await asyncio.sleep(TIME_BETWEEN_ATTEMPTS)
        rclpy.spin_once(node_fixture['node'], timeout_sec=0)
    assert name in get_topics(node_fixture) and name in get_services(node_fixture)


@remapping_test(cli_args=('/fully/qualified/name:=/remapped/s{random_string}',))
async def test_topic_and_service_replacement(node_fixture):
    name = '/remapped/s{random_string}'.format(**node_fixture)

    for attempt in range(ATTEMPTS):
        if name in get_topics(node_fixture) and name in get_services(node_fixture):
            break
        await asyncio.sleep(TIME_BETWEEN_ATTEMPTS)
        rclpy.spin_once(node_fixture['node'], timeout_sec=0)
    assert name in get_topics(node_fixture) and name in get_services(node_fixture)


@remapping_test(cli_args=('rostopic://~/private/name:=/remapped/s{random_string}',))
async def test_topic_replacement(node_fixture):
    name = '/remapped/s{random_string}'.format(**node_fixture)

    for attempt in range(ATTEMPTS):
        if name in get_topics(node_fixture):
            break
        await asyncio.sleep(TIME_BETWEEN_ATTEMPTS)
        rclpy.spin_once(node_fixture['node'], timeout_sec=0)
    assert name in get_topics(node_fixture) and name not in get_services(node_fixture)


@remapping_test(cli_args=('rosservice://~/private/name:=/remapped/s{random_string}',))
async def test_service_replacement(node_fixture):
    name = '/remapped/s{random_string}'.format(**node_fixture)

    for attempt in range(ATTEMPTS):
        if name in get_services(node_fixture):
            break
        await asyncio.sleep(TIME_BETWEEN_ATTEMPTS)
        rclpy.spin_once(node_fixture['node'], timeout_sec=0)
    assert name not in get_topics(node_fixture) and name in get_services(node_fixture)
