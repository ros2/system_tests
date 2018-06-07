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

import pytest
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy

from .utils import launch_process_and_coroutine
from .utils import NamedTemporaryFile
from .utils import require_environment_variable


CLIENT_LIBRARY_EXECUTABLES = (
    require_environment_variable('INITIAL_PARAMS_RCLCPP'),
)


@pytest.fixture(scope='module', params=CLIENT_LIBRARY_EXECUTABLES)
def node_fixture(request):
    """Create a fixture with a node and helper executable."""
    rclpy.init()
    node = rclpy.create_node('tests_yaml')
    try:
        yield {
            'node': node,
            'executable': request.param,
        }
    finally:
        node.destroy_node()
        rclpy.shutdown()


def get_params_test(node, check_func, param_names, *, attempts=10, time_between_attempts=1.0):
    """Make a test that gets params and calls check_func(results)."""
    async def coroutine_test():
        nonlocal node
        nonlocal check_func
        nonlocal param_names
        nonlocal attempts
        nonlocal time_between_attempts
        # wait for service to be ready
        client = node.create_client(GetParameters, '/initial_params_node/get_parameters')
        while not client.service_is_ready() and attempts > 0:
            attempts -= 1
            await asyncio.sleep(time_between_attempts)

        assert attempts > 0

        # Call the service and get the results
        request = GetParameters.Request()
        request.names = param_names
        future = client.call_async(request)
        for _ in range(attempts):
            if future.done():
                break
            rclpy.spin_once(node)
            await asyncio.sleep(time_between_attempts)
        if future.exception():
            raise future.exception()

        assert attempts > 0

        # see if the result looks ok
        check_func(future.result())

    return coroutine_test


def test_bool_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_BOOL == resp.values[0].type
        assert ParameterType.PARAMETER_BOOL == resp.values[1].type
        assert not resp.values[0].bool_value
        assert resp.values[1].bool_value

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        b1: False
        b2: True
""")
        yaml_file.flush()
        # close so it can be opened again on windows
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('b1', 'b2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_integer_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_INTEGER == resp.values[0].type
        assert ParameterType.PARAMETER_INTEGER == resp.values[1].type
        assert 42 == resp.values[0].integer_value
        assert -27 == resp.values[1].integer_value

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        i1: 42
        i2: -27
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('i1', 'i2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_double_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_DOUBLE == resp.values[0].type
        assert ParameterType.PARAMETER_DOUBLE == resp.values[1].type
        assert pytest.approx(3.14) == resp.values[0].double_value
        assert pytest.approx(-2.718) == resp.values[1].double_value

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        d1: 3.14
        d2: -2.718
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('d1', 'd2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_string_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_STRING == resp.values[0].type
        assert ParameterType.PARAMETER_STRING == resp.values[1].type
        assert resp.values[0].string_value == 'hello'
        assert resp.values[1].string_value == 'world'

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        s1: hello
        s2: world
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('s1', 's2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


# TODO(sloretz) PARAMETER_BYTE_ARRAY when rcl_yaml_param_parser supports it


def test_bool_array_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_BOOL_ARRAY == resp.values[0].type
        assert ParameterType.PARAMETER_BOOL_ARRAY == resp.values[1].type
        assert resp.values[0].bool_array_value == [True, False]
        assert resp.values[1].bool_array_value == [False, True]

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        ba1: [true, false]
        ba2: [false, true]
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('ba1', 'ba2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_integer_array_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_INTEGER_ARRAY == resp.values[0].type
        assert ParameterType.PARAMETER_INTEGER_ARRAY == resp.values[1].type
        assert resp.values[0].integer_array_value == [42, -27]
        assert resp.values[1].integer_array_value == [1234, 5678]

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        ia1: [42, -27]
        ia2: [1234, 5678]
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('ia1', 'ia2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_double_array_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_DOUBLE_ARRAY == resp.values[0].type
        assert ParameterType.PARAMETER_DOUBLE_ARRAY == resp.values[1].type
        assert resp.values[0].double_array_value == pytest.approx([3.14, -2.718])
        assert resp.values[1].double_array_value == pytest.approx([1234.5, -9999.0])

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        da1: [3.14, -2.718]
        da2: [1234.5, -9999.0]
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('da1', 'da2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_string_array_params(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 2 == len(resp.values)
        assert ParameterType.PARAMETER_STRING_ARRAY == resp.values[0].type
        assert ParameterType.PARAMETER_STRING_ARRAY == resp.values[1].type
        assert resp.values[0].string_array_value == ['Four', 'score']
        assert resp.values[1].string_array_value == ['and', 'seven']

    with NamedTemporaryFile() as yaml_file:
        yaml_file.write("""
initial_params_node:
    ros__parameters:
        sa1: ['Four', 'score']
        sa2: ['and', 'seven']
""")
        yaml_file.flush()
        yaml_file.close()

        command = (node_fixture['executable'], '__params:=' + yaml_file.name)
        actual_test = get_params_test(node_fixture['node'], check_params, ('sa1', 'sa2'))
        assert 0 == launch_process_and_coroutine(command, actual_test)


def test_multiple_parameter_files(node_fixture):
    def check_params(resp):
        nonlocal node_fixture
        assert 3 == len(resp.values)
        assert ParameterType.PARAMETER_INTEGER == resp.values[0].type
        assert ParameterType.PARAMETER_INTEGER == resp.values[1].type
        assert ParameterType.PARAMETER_INTEGER == resp.values[2].type
        assert 42 == resp.values[0].integer_value
        assert 12345 == resp.values[1].integer_value
        assert -27 == resp.values[2].integer_value

    with NamedTemporaryFile() as first_yaml_file:
        first_yaml_file.write("""
initial_params_node:
    ros__parameters:
        i1: 42
        i2: -27
""")
        first_yaml_file.flush()
        first_yaml_file.close()
        with NamedTemporaryFile() as second_yaml_file:
            second_yaml_file.write("""
initial_params_node:
    ros__parameters:
        i2: 12345
        i3: -27
""")
            second_yaml_file.flush()
            second_yaml_file.close()

            command = (
                node_fixture['executable'],
                '__params:=' + first_yaml_file.name,
                '__params:=' + second_yaml_file.name
            )
            actual_test = get_params_test(node_fixture['node'], check_params, ('i1', 'i2', 'i3'))
            assert 0 == launch_process_and_coroutine(command, actual_test)
