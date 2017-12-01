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


def requester(service_name):
    import rclpy
    from test_msgs.service_fixtures import get_test_srv

    # Import the service
    service_pkg = 'test_msgs'
    module = importlib.import_module(service_pkg + '.srv')
    srv_mod = getattr(module, service_name)

    srv_fixtures = get_test_srv(service_name)
    service_name = 'test_service_' + service_name

    rclpy.init(args=[])
    try:
        node = rclpy.create_node('requester')
        try:
            # wait for the service to be available
            client = node.create_client(srv_mod, service_name)
            tries = 15
            while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0) and tries > 0:
                print('service not available, waiting again...')
                tries -= 1
            assert tries > 0, 'service still not available, aborting test'

            print('requester: beginning request')
            # Make one call to that service
            for req, resp in srv_fixtures:
                client.call(req)
                client.wait_for_future()
                assert repr(client.response) == repr(resp), \
                    'unexpected response %r\n\nwas expecting %r' % (client.response, resp)
                print('received reply #%d of %d' % (
                    srv_fixtures.index([req, resp]) + 1, len(srv_fixtures)))
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('service_name', default='Primitives',
                        help='name of the ROS message')
    args = parser.parse_args()
    try:
        requester(service_name=args.service_name)
    except KeyboardInterrupt:
        print('requester stopped cleanly')
    except BaseException:
        print('exception in requester:', file=sys.stderr)
        raise
