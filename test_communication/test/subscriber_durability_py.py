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

import functools
import importlib
import os
import sys

# this is needed to allow import of test_communication messages
sys.path.insert(0, os.getcwd())


def listener_cb(msg, received_messages, expected_msgs):
    known_msg = False
    for exp in expected_msgs:
        if msg.__repr__() == exp.__repr__():
            print('received message #{} of {}'.format(
                expected_msgs.index(exp) + 1, len(expected_msgs)))
            known_msg = True
            already_received = False
            for rmsg in received_messages:
                if rmsg.__repr__() == msg.__repr__():
                    already_received = True
                    break

            if not already_received:
                received_messages.append(msg)
            break
    if known_msg is False:
        raise RuntimeError('received unexpected message %r' % msg)


def listener():
    from message_fixtures import get_msg_primitives
    import rclpy
    from rclpy.qos import QoSDurabilityPolicy

    message_pkg = 'test_communication'
    module = importlib.import_module(message_pkg + '.msg')
    msg_mod = getattr(module, 'Primitives')

    rclpy.init(args=[])

    node = rclpy.create_node('listener')

    received_messages = []
    expected_msgs = get_msg_primitives()

    chatter_callback = functools.partial(
        listener_cb, received_messages=received_messages, expected_msgs=expected_msgs)

    custom_qos_profile = rclpy.qos.qos_profile_default
    custom_qos_profile.durability = QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL

    node.create_subscription(
        msg_mod, 'test_durability_message_primitives', chatter_callback,
        qos_profile=custom_qos_profile)

    spin_count = 1
    print('subscriber: beginning loop')
    while (rclpy.ok() and len(received_messages) != len(expected_msgs)):
        rclpy.spin_once(node)
        spin_count += 1
        print('spin_count: ' + str(spin_count))
    node.destroy_node()
    rclpy.shutdown()

    assert len(received_messages) == len(expected_msgs),\
        'Should have received {} {} messages from talker'.format(len(expected_msgs), 'Primitive')


if __name__ == '__main__':
    try:
        listener()
    except KeyboardInterrupt:
        print('subscriber stopped cleanly')
    except BaseException:
        print('exception in subscriber:', file=sys.stderr)
        raise
