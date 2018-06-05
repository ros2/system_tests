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
import os
import sys

from launch import LaunchDescriptor
from launch.exit_handler import primary_exit_handler
from launch.launcher import DefaultLauncher


def require_environment_variable(name):
    """Get environment variable or raise if it does not exist."""
    path = os.getenv(name)
    if not path:
        raise EnvironmentError('Missing environment variable "%s"' % name)
    return path


def launch_process_and_coroutine(command, coroutine):
    """Execute a command and coroutine in parallel."""
    # Execute python files using same python used to start this test
    env = dict(os.environ)
    if command[0][-3:] == '.py':
        command.insert(0, sys.executable)
        env['PYTHONUNBUFFERED'] = '1'

    ld = LaunchDescriptor()
    ld.add_process(
        cmd=command,
        name='helper_for_' + coroutine.__name__,
        env=env
    )
    ld.add_coroutine(
        coroutine(),
        name=coroutine.__name__,
        exit_handler=primary_exit_handler
    )
    launcher = DefaultLauncher()
    launcher.add_launch_descriptor(ld)
    return_code = launcher.launch()
    return return_code


def make_coroutine_test(*, check_func, attempts=10, time_between_attempts=1.0):
    """Make a test that succeeds when check_func() returns True."""
    async def coroutine_test():
        for attempt in range(attempts):
            if check_func():
                # Test passed
                return
            await asyncio.sleep(time_between_attempts)
        # final attempt to check condition
        assert check_func()

    return coroutine_test
