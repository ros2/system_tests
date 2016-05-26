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

# this is needed to allow rclpy to be imported from the build folder
sys.path.insert(0, os.getcwd())
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.getcwd())), 'rclpy'))


def fill_msg(msg, message_name, i):
    if 'Builtins' == message_name:
        msg.duration_value.sec = int(i)
        msg.duration_value.nanosec = int(100 * i)
        msg.time_value.sec = int(i)
        msg.time_value.nanosec = int(1000 * i)
    elif 'Primitives' == message_name:
        msg.bool_value = True
        msg.byte_value = bytes([i])
        msg.char_value = str(i)[0]
        msg.float32_value = float(i)
        msg.float64_value = float(i)
        msg.int8_value = int(-i)
        msg.uint8_value = int(i)
        msg.int16_value = int(-i)
        msg.uint16_value = int(i)
        msg.int32_value = int(-i)
        msg.uint32_value = int(i)
        msg.int64_value = int(-i)
        msg.uint64_value = int(i)
        msg.string_value = str(i)
    elif 'FieldsWithSameType' == message_name:
        msg.primitive_values1.bool_value = True
        msg.primitive_values1.byte_value = bytes([i])
        msg.primitive_values1.char_value = str(i)[0]
        msg.primitive_values1.float32_value = float(i)
        msg.primitive_values1.float64_value = float(i)
        msg.primitive_values1.int8_value = int(-i)
        msg.primitive_values1.uint8_value = int(i)
        msg.primitive_values1.int16_value = int(-i)
        msg.primitive_values1.uint16_value = int(i)
        msg.primitive_values1.int32_value = int(-i)
        msg.primitive_values1.uint32_value = int(i)
        msg.primitive_values1.int64_value = int(-i)
        msg.primitive_values1.uint64_value = int(i)
        msg.primitive_values1.string_value = str(i)
        msg.primitive_values2.bool_value = False
        msg.primitive_values2.byte_value = bytes([i])
        msg.primitive_values2.char_value = str(i)[0]
        msg.primitive_values2.float32_value = float(i + 1)
        msg.primitive_values2.float64_value = float(i + 1)
        msg.primitive_values2.int8_value = int(-(i + 1))
        msg.primitive_values2.uint8_value = int(i + 1)
        msg.primitive_values2.int16_value = int(-(i + 1))
        msg.primitive_values2.uint16_value = int(i + 1)
        msg.primitive_values2.int32_value = int(-(i + 1))
        msg.primitive_values2.uint32_value = int(i + 1)
        msg.primitive_values2.int64_value = int(-(i + 1))
        msg.primitive_values2.uint64_value = int(i + 1)
        msg.primitive_values2.string_value = str(i + 1)
    elif 'StaticArrayNested' == message_name:
        for j in range(len(msg.primitive_values)):
            msg.primitive_values[j].bool_value = True
            msg.primitive_values[j].byte_value = bytes([i + j])
            msg.primitive_values[j].char_value = str(i + j)[0]
            msg.primitive_values[j].float32_value = float(i + j)
            msg.primitive_values[j].float64_value = float(i + j)
            msg.primitive_values[j].int8_value = int(-(i + j))
            msg.primitive_values[j].uint8_value = int(i + j)
            msg.primitive_values[j].int16_value = int(-(i + j))
            msg.primitive_values[j].uint16_value = int(i + j)
            msg.primitive_values[j].int32_value = int(-(i + j))
            msg.primitive_values[j].uint32_value = int(i + j)
            msg.primitive_values[j].int64_value = int(-(i + j))
            msg.primitive_values[j].uint64_value = int(i + j)
            msg.primitive_values[j].string_value = str(i + j)
    elif 'StaticArrayPrimitives' == message_name:
        msg.bool_values = [True for x in range(len(msg.bool_values))]
        msg.char_values = [str(i + x)[0] for x in range(len(msg.char_values))]
        msg.byte_values = [bytes([i + x]) for x in range(len(msg.byte_values))]
        msg.float32_values = [float(i + x) for x in range(len(msg.float32_values))]
        msg.float64_values = [float(i + x) for x in range(len(msg.float64_values))]
        msg.int8_values = [int(-(i + x)) for x in range(len(msg.int8_values))]
        msg.uint8_values = [int(i + x) for x in range(len(msg.uint8_values))]
        msg.int16_values = [int(-(i + x)) for x in range(len(msg.int16_values))]
        msg.uint16_values = [int(i + x) for x in range(len(msg.uint16_values))]
        msg.int32_values = [int(-(i + x)) for x in range(len(msg.int32_values))]
        msg.uint32_values = [int(i + x) for x in range(len(msg.uint32_values))]
        msg.int64_values = [int(-(i + x)) for x in range(len(msg.int64_values))]
        msg.uint64_values = [int(i + x) for x in range(len(msg.uint64_values))]
        msg.string_values = [str(i + x) for x in range(len(msg.string_values))]
    elif 'DynamicArrayPrimitives' == message_name:
        msg.bool_values = [True for x in range(i + 3)]
        msg.char_values = [str(i + x)[0] for x in range(i + 3)]
        msg.byte_values = [bytes([i + x]) for x in range(i + 3)]
        msg.float32_values = [float(i + x) for x in range(i + 3)]
        msg.float64_values = [float(i + x) for x in range(i + 3)]
        msg.int8_values = [int(-(i + x)) for x in range(i + 3)]
        msg.uint8_values = [int(i + x) for x in range(i + 3)]
        msg.int16_values = [int(-(i + x)) for x in range(i + 3)]
        msg.uint16_values = [int(i + x) for x in range(i + 3)]
        msg.int32_values = [int(-(i + x)) for x in range(i + 3)]
        msg.uint32_values = [int(i + x) for x in range(i + 3)]
        msg.int64_values = [int(-(i + x)) for x in range(i + 3)]
        msg.uint64_values = [int(i + x) for x in range(i + 3)]
        msg.string_values = [str(i + x) for x in range(i + 3)]
    elif 'DynamicArrayNested' == message_name:
        msg.primitive_values = []
        from test_communication.msg import Primitives
        for x in range(i + 3):
            tmpprimitive = Primitives()
            tmpprimitive.bool_value = True
            tmpprimitive.byte_value = bytes([x])
            tmpprimitive.char_value = str(x)[0]
            tmpprimitive.float32_value = float(x)
            tmpprimitive.float64_value = float(x)
            tmpprimitive.int8_value = int(-x)
            tmpprimitive.uint8_value = int(x)
            tmpprimitive.int16_value = int(-x)
            tmpprimitive.uint16_value = int(x)
            tmpprimitive.int32_value = int(-x)
            tmpprimitive.uint32_value = int(x)
            tmpprimitive.int64_value = int(-x)
            tmpprimitive.uint64_value = int(x)
            tmpprimitive.string_value = str(x)
            msg.primitive_values.append(tmpprimitive)
    return msg


def talker(message_pkg, message_name, number_of_cycles):
    import rclpy
    from rclpy.qos import qos_profile_default

    rclpy.init([])

    # TODO(wjwwood) move this import back to the module level when
    # it is possible to import the messages before rclpy.init().
    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, message_name)
    assert msg_mod.__class__._TYPE_SUPPORT is not None

    node = rclpy.create_node('talker')

    chatter_pub = node.create_publisher(msg_mod, 'chatter', qos_profile_default)

    msg = msg_mod()

    msg_count = 1
    print('talker: beginning loop')
    while rclpy.ok() and msg_count < number_of_cycles:
        msg = fill_msg(msg, message_name, msg_count)
        msg_count += 1
        chatter_pub.publish(msg)
        print('talker sending: %r' % msg)
        time.sleep(1)
    rclpy.shutdown()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-p', '--message_pkg', default='test_communication',
                        help='name of the message package')
    parser.add_argument('-m', '--message_name', default='Primitives',
                        help='name of the ROS message')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=5,
                        help='number of sending attempts')
    args = parser.parse_args()
    try:
        talker(
            message_pkg=args.message_pkg,
            message_name=args.message_name,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('talker stopped cleanly')
    except BaseException:
        print('exception in talker:', file=sys.stderr)
        raise
