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

import rclpy, sys
from rclpy.node import Node

from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self, nmr):
        super().__init__('minimal_subscriber')
        self.timer_n = 3
        self.priority = 1
        self.nmr = nmr

        # create a subscription for each publisher
        for i in range(nmr):
          self.init_subscription(self.timer_n, topics_info[i+1])


    def init_subscription(self, timer, topic):

      self.subscription = self.create_subscription(
            String,
            topic['name'],
            lambda msg: self.get_logger().info('I heard: "%s"' % msg.data) if topic['priority'] == self.priority else False
            , 10)

      if self.priority == topic['priority']:
        self.timer = self.create_timer(timer, self.timer_callback)
      self.subscription

      # The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.
      # The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.


    def timer_callback(self):

      if self.priority == self.nmr:
        self.priority = 1
      else:
        self.priority += 1


def main(args=None):
    rclpy.init(args=args)

    nmr_topics = int(sys.argv[1])
    global topics_info
    topics_info = {}

    # Archaic Method :: topic names must start with tag topic_
    while nmr_topics > 0:
      i = (int(sys.argv[1]) - nmr_topics) + 1
      topics_info[i] = {}
      topics_info[i]['priority'] = i
      topics_info[i]['name'] = 'topic_' + str(i)
      nmr_topics -= 1

    minimal_subscriber = MinimalSubscriber(int(sys.argv[1]))

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
