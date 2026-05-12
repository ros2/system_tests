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
Launch test: test backend publisher to plain CPU subscriber over FastRTPS.

The subscriber is not given any backend-related SubscriptionOptions, so it
advertises only the default "cpu" backend. The publisher must therefore
fall back to CPU serialization and the subscriber must receive a CPU-backed
buffer. Data content is still verified byte-for-byte.
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

    # No acceptable_buffer_backends override — default subscription options.
    subscriber_node = Node(
        package='test_rosidl_buffer',
        executable='test_backend_subscriber',
        name='test_backend_subscriber',
        output='screen',
        parameters=[{
            'topic_name': 'test_bytes',
            'expected_backend': 'cpu',
        }],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp'),
        SetEnvironmentVariable('RCL_ASSERT_RMW_ID_MATCHES', 'rmw_fastrtps_cpp'),
        publisher_node,
        subscriber_node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestTestToCpuFastRTPS(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_test_to_cpu')
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

    def test_test_to_cpu_falls_back(self):
        self.assertTrue(
            self._spin_until(target=20, timeout_sec=20.0),
            f'Only received {self.subscriber_count} messages')
        self.assertTrue(self.validation_passed, 'Subscriber validation failed')
        self.assertEqual(
            'cpu', self.observed_backend,
            'With a default-configured CPU subscriber, the subscriber must '
            'observe backend_type == "cpu". Got '
            f'{self.observed_backend!r}.')


@launch_testing.post_shutdown_test()
class TestTestToCpuFastRTPSShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
