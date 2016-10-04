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
import json
import time
import sys


def rostopic_list(wait_time, result_string):

    import rclpy

    result_dictionary = json.loads(result_string)
    rclpy.init()

    node = rclpy.create_node('rostopic_list')
    time.sleep(wait_time)
    if(rclpy.ok()):
        a = node.get_topic_names_and_types()
    rclpy.shutdown()
    assert a.topic_count == len(result_dictionary), \
        'a.topic_count: {} != len(dictionary_result) : {}'.format(a.topic_count, len(result_dictionary))
    i = 0
    for topic_name in a.topic_names:
        assert a.type_names[i] == result_dictionary[topic_name], \
            'expected msg type {} but got {} instead'.format(
                result_dictionary[topic_name], a.type_names[i])
        i += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-r', '--result_string', default='{}',
                        help='dictionnaire of expected topic names and type pairs')
    parser.add_argument('-w', '--wait_time', type=float, default=2.0,
                        help='time to wait for discovery')
    args = parser.parse_args()
    try:
        rostopic_list(
            wait_time=args.wait_time,
            result_string=args.result_string)
    except KeyboardInterrupt:
        print('rostopic stopped cleanly')
    except BaseException:
        print('exception in rostopic:', file=sys.stderr)
        raise
