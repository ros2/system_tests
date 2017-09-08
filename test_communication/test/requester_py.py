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


def requester(service_name, number_of_cycles):
    from test_msgs.service_fixtures import get_test_srv
    import rclpy

    service_pkg = 'test_msgs'
    module = importlib.import_module(service_pkg + '.srv')
    srv_mod = getattr(module, service_name)

    rclpy.init(args=[])

    node = rclpy.create_node('requester')

    srv_fixtures = get_test_srv(service_name)

    client = node.create_client(srv_mod, 'test_service_' + service_name)

    spin_count = 1
    received_replies = []
    print('requester: beginning loop')
    while rclpy.ok() and spin_count < number_of_cycles:
        for req, resp in srv_fixtures:
            client.call(req)
            client.wait_for_future()
            assert repr(client.response) == repr(resp), \
                'received unexpected response %r\n\nwas expecting %r' % (client.response, resp)
            print('received reply #%d of %d' %
                  (srv_fixtures.index([req, resp]) + 1, len(srv_fixtures)))
            received_replies.append(resp)
            spin_count += 1
            print('spin_count: ' + str(spin_count))
        break
    node.destroy_node()
    rclpy.shutdown()
    assert len(received_replies) == len(srv_fixtures), \
        'Should have received %d responsed from replier' % len(srv_fixtures)
    print('everything went well !')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('service_name', default='Primitives',
                        help='name of the ROS message')
    parser.add_argument('-n', '--number_of_cycles', type=int, default=10,
                        help='number of sending attempts')
    args = parser.parse_args()
    try:
        requester(
            service_name=args.service_name,
            number_of_cycles=args.number_of_cycles)
    except KeyboardInterrupt:
        print('requester stopped cleanly')
    except BaseException:
        print('exception in requester:', file=sys.stderr)
        raise
