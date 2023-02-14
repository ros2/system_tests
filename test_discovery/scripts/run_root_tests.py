#!/usr/bin/env python3

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

import argparse
import os
import sys

from ament_index_python import get_package_share_path
from ament_index_python import get_resources


def get_rmw_implementations():
    resources = list(get_resources('rmw_typesupport').keys())
    if 'rmw_implementation' in resources:
        resources.remove('rmw_implementation')
    return tuple(resources)


def get_tests_dir():
    pkg_path = get_package_share_path('test_discovery')
    return pkg_path / 'roottests'


def get_workspaces():
    # Get an ordered list of workspaces that are sourced
    prefixes = os.environ['AMENT_PREFIX_PATH']
    if not prefixes:
        raise ValueError('No ROS/Colcon workspace sourced')
    workspaces = set()
    for prefix in prefixes.split(':'):
        if not prefix:
            # env var might have began or ended with a ':'
            continue
        # If there exists a parent folder containing a setup.bash
        # then assume this is an isolated colcon workspace
        if os.path.exists(os.path.join(prefix, '../setup.bash')):
            workspaces.add(os.path.dirname(prefix))
        else:
            # Assume a merged ament/colcon workspace
            workspaces.add(prefix)
    return tuple(workspaces)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rmw', default=None)
    parser.add_argument('--select', default=None)
    args = parser.parse_args()

    rmw_implementations = get_rmw_implementations()
    if args.rmw:
        if args.rmw not in rmw_implementations:
            raise ValueError(f'{args.rmw} is not an installed rmw: {rmw_implementations}')
        rmw_implementations = [args.rmw]

    cmd = []
    cmd.append('sudo')
    cmd.append(sys.executable)
    cmd.append('-m')
    cmd.append('pytest')
    cmd.append('-c')
    cmd.append(str(get_tests_dir() / 'conftest.py'))
    if args.select:
        cmd.append('-k')
        cmd.append(args.select)
    cmd.append(f'--rmws={":".join(rmw_implementations)}')
    cmd.append(f'--ros-workspaces={":".join(get_workspaces())}')
    cmd.append(str(get_tests_dir() / 'test_discovery.py'))

    print('Executing the following command:')
    print('================================')
    print('$', *cmd)
    print('================================')

    os.execvp(cmd[0], cmd)


if __name__ == '__main__':
    main()
