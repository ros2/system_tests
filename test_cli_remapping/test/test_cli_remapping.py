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

import os
import random
import sys
import threading
import time

from launch import LaunchDescriptor
from launch.launcher import DefaultLauncher
import pytest
import rclpy


class LaunchHelper:
    """Manage boilerplate for launching and tearing down a helper process."""

    def __init__(self, command, name):
        self._thread = None
        self._rc = None
        self._launcher = None
        self._run_finished = threading.Event()

        # Execute python files using same python used to start this test
        env = dict(os.environ)
        if command[0][-3:] == '.py':
            command.insert(0, sys.executable)
            env['PYTHONUNBUFFERED'] = '1'

        ld = LaunchDescriptor()
        ld.add_process(
            cmd=command,
            name=name,
            env=env
        )
        self._launcher = DefaultLauncher()
        self._launcher.add_launch_descriptor(ld)

    def __enter__(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        self._rc = self._launcher.launch()
        self._run_finished.set()

    def __exit__(self, type_, value, traceback):
        self._launcher.interrupt_launch()
        self._launcher.wait_on_launch_to_finish(timeout=None)
        self._run_finished.wait()
        assert self._rc == 0, 'Launch failed with exit code %r' % (self._rc)


def get_environment_variable(name):
    """Get enironment variable or raise if it does not exist."""
    path = os.getenv(name)
    if not path:
        raise EnvironmentError('Missing environment variable %r' % name)
    return path


CLIENT_LIBRARY_EXECUTABLES = (
    get_environment_variable('NAME_MAKER_RCLCPP'),
    get_environment_variable('NAME_MAKER_RCLPY')
)


@pytest.fixture(scope='module', params=CLIENT_LIBRARY_EXECUTABLES)
def node_fixture(request):
    """Create a node instance all tests in this file will use."""
    rclpy.init()
    node = rclpy.create_node('test_cli_remapping')
    try:
        yield {
            'node': node,
            'executable': request.param
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def get_topics(node_fixture):
    topic_names_and_types = node_fixture['node'].get_topic_names_and_types()
    return [name for name, _ in topic_names_and_types]


def get_services(node_fixture):
    service_names_and_types = node_fixture['node'].get_service_names_and_types()
    return [name for name, _ in service_names_and_types]


ATTEMPTS = 10
TIME_BETWEEN_ATTEMPTS = 1


def test_node_name_replacement(node_fixture):
    node_name = 'node_%d_%s' % (random.randint(0, 9999), time.strftime('%H_%M_%S', time.gmtime()))

    command = [node_fixture['executable'], '__node:=' + node_name]
    with LaunchHelper(command, 'test_node_name_replacement'):
        for attempt in range(ATTEMPTS):
            if node_name in node_fixture['node'].get_node_names():
                break
            time.sleep(TIME_BETWEEN_ATTEMPTS)
            rclpy.spin_once(node_fixture['node'], timeout_sec=0)
        assert node_name in node_fixture['node'].get_node_names()


def test_namespace_replacement(node_fixture):
    namespace = '/ns/_%d_%s' % (random.randint(0, 9999), time.strftime('%H_%M_%S', time.gmtime()))
    name = namespace + '/relative/name'

    command = [node_fixture['executable'], '__ns:=' + namespace]
    with LaunchHelper(command, 'test_namespace_replacement'):
        for attempt in range(ATTEMPTS):
            if name in get_topics(node_fixture) and name in get_services(node_fixture):
                break
            time.sleep(TIME_BETWEEN_ATTEMPTS)
            rclpy.spin_once(node_fixture['node'], timeout_sec=0)
        assert name in get_topics(node_fixture) and name in get_services(node_fixture)


def test_topic_and_service_replacement(node_fixture):
    name = '/remapped/_%d_%s' % (random.randint(0, 9999), time.strftime('%H_%M_%S', time.gmtime()))

    command = [node_fixture['executable'], '/fully/qualified/name:=' + name]
    with LaunchHelper(command, 'test_topic_and_service_replacement'):
        for attempt in range(ATTEMPTS):
            if name in get_topics(node_fixture) and name in get_services(node_fixture):
                break
            time.sleep(TIME_BETWEEN_ATTEMPTS)
            rclpy.spin_once(node_fixture['node'], timeout_sec=0)
        assert name in get_topics(node_fixture) and name in get_services(node_fixture)


def test_topic_replacement(node_fixture):
    name = '/remapped/_%d_%s' % (random.randint(0, 9999), time.strftime('%H_%M_%S', time.gmtime()))

    command = [node_fixture['executable'], 'rostopic://~/private/name:=' + name]
    with LaunchHelper(command, 'test_topic_replacement'):
        for attempt in range(ATTEMPTS):
            if name in get_topics(node_fixture):
                break
            time.sleep(TIME_BETWEEN_ATTEMPTS)
            rclpy.spin_once(node_fixture['node'], timeout_sec=0)
        assert name in get_topics(node_fixture) and name not in get_services(node_fixture)


def test_service_replacement(node_fixture):
    name = '/remapped/_%d_%s' % (random.randint(0, 9999), time.strftime('%H_%M_%S', time.gmtime()))

    command = [node_fixture['executable'], 'rosservice://~/private/name:=' + name]
    with LaunchHelper(command, 'test_service_replacement'):
        for attempt in range(ATTEMPTS):
            if name in get_services(node_fixture):
                break
            time.sleep(TIME_BETWEEN_ATTEMPTS)
            rclpy.spin_once(node_fixture['node'], timeout_sec=0)
        assert name not in get_topics(node_fixture) and name in get_services(node_fixture)
