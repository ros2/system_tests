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
Launch test: test backend publisher to test backend subscriber over FastRTPS.

Verifies that:
  * The subscriber actually receives messages.
  * Every received message reports backend_type == "test" (descriptor path).
  * The publisher's TestBufferImpl::to_cpu() was never invoked, proving the
    buffer was handed directly to the RMW via the backend descriptor rather
    than converted to CPU bytes first.
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
        executable='test_backend_publisher',
        name='test_backend_publisher',
        output='screen',
        parameters=[{
            'backend_mode': 'test',
            'topic_name': 'test_bytes',
            'publish_rate_ms': 100,
            'max_publish_count': 40,
        }],
    )

    subscriber_node = Node(
        package='test_rosidl_buffer',
        executable='test_backend_subscriber',
        name='test_backend_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_bytes',
            'expected_backend': 'test',
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


class TestTestToTestFastRTPS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_test_to_test')
        self.subscriber_count = 0
        self.validation_passed = None
        self.observed_backend = None
        self.publisher_to_cpu = None

        self.node.create_subscription(
            UInt32, 'subscriber_count', self._on_sub_count, 10)
        self.node.create_subscription(
            Bool, 'validation_result', self._on_validation, 10)
        self.node.create_subscription(
            String, 'observed_backend_type', self._on_backend, 10)
        self.node.create_subscription(
            UInt32, 'publisher_to_cpu_count', self._on_to_cpu, 10)

    def tearDown(self):
        self.node.destroy_node()

    def _on_sub_count(self, msg):
        self.subscriber_count = msg.data

    def _on_validation(self, msg):
        self.validation_passed = msg.data

    def _on_backend(self, msg):
        self.observed_backend = msg.data

    def _on_to_cpu(self, msg):
        self.publisher_to_cpu = msg.data

    def _spin_until(self, target=20, timeout_sec=20.0):
        start = time.time()
        while (
            (self.subscriber_count < target
             or self.validation_passed is None
             or self.observed_backend is None
             or self.publisher_to_cpu is None)
            and time.time() - start < timeout_sec
        ):
            rclpy.spin_once(self.node, timeout_sec=0.1)
        return self.subscriber_count >= target

    def test_test_to_test_uses_descriptor_path(self):
        self.assertTrue(
            self._spin_until(target=20, timeout_sec=20.0),
            f'Only received {self.subscriber_count} messages')
        self.assertTrue(self.validation_passed, 'Subscriber validation failed')
        self.assertEqual(
            'test', self.observed_backend,
            'Subscriber must observe backend_type == "test"; got '
            f'{self.observed_backend!r}. A "cpu" value means the serialization '
            'layer fell back to CPU instead of going through '
            'TestBufferBackend::from_descriptor_with_endpoint.')
        self.assertEqual(
            0, self.publisher_to_cpu,
            'TestBufferImpl::to_cpu() must NEVER be called in the test→test '
            f'scenario. Got {self.publisher_to_cpu} calls. A non-zero value '
            'means the RMW fell back to CPU serialization on the publisher '
            'side instead of using the backend descriptor path.')


@launch_testing.post_shutdown_test()
class TestTestToTestFastRTPSShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
