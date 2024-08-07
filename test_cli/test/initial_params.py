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

import sys

import rclpy
from rclpy.executors import ExternalShutdownException


def main(argv=sys.argv):
    try:
        with rclpy.init(args=argv):
            node = rclpy.create_node(
                'initial_params_node',
                namespace='/',
                allow_undeclared_parameters=True,
                automatically_declare_parameters_from_overrides=True)
            rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    return 0


if __name__ == '__main__':
    sys.exit(main())
