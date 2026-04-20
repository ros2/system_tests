# Copyright 2026 Open Source Robotics Foundation, Inc.
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

"""
Launch test: nested ByteArray messages over FastRTPS.

Verifies that a message containing ByteArray[] round-trips nested uint8[]
fields correctly when the publisher constructs each nested buffer with the
test backend.
"""

import time
import unittest

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from std_msgs.msg import Bool, String, UInt32


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    publisher_node = Node(
        package='test_rosidl_buffer',
        executable='test_backend_nested_msgs_publisher',
        name='test_backend_nested_msgs_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'test',
            'topic_name': 'test_byte_array_list',
            'publish_rate_ms': 100,
            'max_publish_count': 40,
        }],
    )

    subscriber_node = Node(
        package='test_rosidl_buffer',
        executable='test_backend_nested_msgs_subscriber',
        name='test_backend_nested_msgs_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_byte_array_list',
            'acceptable_buffer_backends': 'any',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RCL_ASSERT_RMW_ID_MATCHES', 'rmw_fastrtps_cpp'),
        publisher_node,
        subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestNestedMsgsFastRTPS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_nested_msgs')
        self.subscriber_count = 0
        self.validation_passed = None
        self.observed_backend = None

        self.node.create_subscription(
            UInt32, 'subscriber_count', self._on_sub_count, 10)
        self.node.create_subscription(
            Bool, 'validation_result', self._on_validation, 10)
        self.node.create_subscription(
            String, 'observed_backend_type', self._on_backend, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_sub_count(self, msg):
        self.subscriber_count = msg.data

    def _on_validation(self, msg):
        self.validation_passed = msg.data

    def _on_backend(self, msg):
        self.observed_backend = msg.data

    def _spin_until(self, target=20, timeout_sec=20.0):
        start = time.time()
        while (
            (self.subscriber_count < target
             or self.validation_passed is None
             or self.observed_backend is None)
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target

    def test_nested_byte_arrays_round_trip(self):
        self.assertTrue(
            self._spin_until(target=20, timeout_sec=20.0),
            f'Only received {self.subscriber_count} messages')
        self.assertTrue(self.validation_passed, 'Subscriber validation failed')
        self.assertIn(
            self.observed_backend, ('test', 'cpu'),
            f'Unexpected nested buffer backend: {self.observed_backend!r}')


@launch_testing.post_shutdown_test()
class TestNestedMsgsFastRTPSShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
