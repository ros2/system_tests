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
import sys

import rclpy
from rclpy.node import Node
from test_msgs.service_fixtures import get_test_srv


class Requester(Node):

    def __init__(self, service_name):
        super().__init__('requester')
        # Import the service
        service_pkg = 'test_msgs'
        module = importlib.import_module(service_pkg + '.srv')
        self.srv_mod = getattr(module, service_name)

        self.srv_fixtures = get_test_srv(service_name)
        self.service_name = 'test_service_' + service_name

    def run(self):
        # wait for the service to be available
        client = node.create_client(self.srv_mod, self.service_name)
        tries = 15
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0) and tries > 0:
            print('service not available, waiting again...')
            tries -= 1
        assert tries > 0

        print('requester: beginning request')
        # Make one call to that service
        for req, resp in self.srv_fixtures:
            client.call(req)
            client.wait_for_future()
            assert repr(client.response) == repr(resp), \
                'unexpected response %r\n\nwas expecting %r' % (client.response, resp)
            print('received reply #%d of %d' % (
                self.srv_fixtures.index([req, resp]) + 1, len(self.srv_fixtures)))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('service_name', default='Primitives',
                        help='name of the ROS message')
    args = parser.parse_args()
    try:
        rclpy.init(args=[])
        node = Requester(args.service_name)
        try:
            node.run()
        finally:
            node.destroy_node()
    except KeyboardInterrupt:
        print('requester stopped cleanly')
    except BaseException:
        print('exception in requester:', file=sys.stderr)
        raise
    finally:
        rclpy.shutdown()
