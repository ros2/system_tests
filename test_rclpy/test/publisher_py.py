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

# this is needed to allow import of test_rclpy messages
sys.path.insert(0, os.getcwd())


def talker(message_name, topic_name, number_of_cycles):
    import rclpy
    from rclpy.impl.rmw_implementation_tools import select_rmw_implementation
    from rclpy.impl.rmw_implementation_tools import get_rmw_implementations
    from rclpy.qos import qos_profile_default

    # print('message_name: ' + message_name)
    # print('topic_name: ' + topic_name)
    module = importlib.import_module('test_rclpy.msg')
    msg_mod = getattr(module, message_name)

    rmw_implementations = get_rmw_implementations()
    assert(os.environ.get('RCLPY_IMPLEMENTATION', 'Not Set') in rmw_implementations)
    select_rmw_implementation(os.environ['RCLPY_IMPLEMENTATION'])

    rclpy.init()

    node = rclpy.create_node('talker_' + topic_name)

    chatter_pub = node.create_publisher(
        msg_mod, topic_name, qos_profile_default)

    cycle_count = 0
    # print('talker: beginning loop')
    msg = msg_mod()
    while rclpy.ok() and cycle_count < number_of_cycles:
        cycle_count += 1
        chatter_pub.publish(msg)
        time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('message_name', default='Primitives',
                        choices=['Primitives', 'DynamicArrayPrimitives'],
                        help='name of the ROS message')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=15,
                        help='number of sending attempts')
    parser.add_argument('-t', '--topic_name', type=str, default='talker',
                        help='name of the advertised topic')
    args = parser.parse_args()
    try:
        talker(
            message_name=args.message_name,
            topic_name=args.topic_name,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
