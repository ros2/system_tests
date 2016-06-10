# Copyright 2016 Open Source Robotics Foundation, Inc.
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
import importlib
import os
import sys
import time

# this is needed to allow import of test_communication messages
sys.path.insert(0, os.getcwd())


def talker(message_name, rmw_implementation, number_of_cycles):
    import rclpy
    from rclpy.qos import qos_profile_default
    from rclpy.impl.rmw_implementation_tools import select_rmw_implementation

    select_rmw_implementation(rmw_implementation)

    rclpy.init([])

    message_pkg = 'test_communication'
    # TODO(wjwwood) move this import back to the module level when
    # it is possible to import the messages before rclpy.init().
    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)
    assert msg_mod.__class__._TYPE_SUPPORT is not None
    from message_fixtures import get_test_msg

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(
        msg_mod, 'test_message_' + message_name, qos_profile_default)

    cycle_count = 0
    print('talker: beginning loop')
    msgs = get_test_msg(message_name)
    while rclpy.ok() and cycle_count < number_of_cycles:
        cycle_count += 1
        msg_count = 0
        for msg in msgs:
            chatter_pub.publish(msg)
            msg_count += 1
            print('publishing message #%d' % msg_count)
            time.sleep(0.1)
        time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations
    rmw_implementations = get_rmw_implementations()
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('message_name', default='Primitives',
                        help='name of the ROS message')
    parser.add_argument('-r', '--rmw_implementation', default=rmw_implementations[0],
                        choices=rmw_implementations,
                        help='rmw_implementation identifier')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=10,
                        help='number of sending attempts')
    args = parser.parse_args()
    try:
        talker(
            message_name=args.message_name,
            rmw_implementation=args.rmw_implementation,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
