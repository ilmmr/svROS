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

import rclpy, sys, os, re
from rclpy.node import Node

from std_msgs.msg import String
from interfaces.msg import PubSub

class MinimalSubscriber(Node):

    def __init__(self, topic, nmr_pubs):
        super().__init__('minimal_subscriber')
        self.timer_n = 3
        self.priority = 1
        self.nmr_pubs = nmr_pubs
        self.topic = topic
        # self.nmr = nmr

        # create a subscription for each publisher
        # for i in range(nmr):
          # self.init_subscription(self.timer_n, topics_info[i+1])
        self.init_subscription(self.timer_n, topic)

    def init_subscription(self, timer, topic):

      self.subscription = self.create_subscription(
            PubSub,
            topic,
            self.callback, 10)

      self.timer = self.create_timer(timer, self.timer_callback)
      self.subscription

      # The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.
      # The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.

    # subscription callback
    def callback(self, msg):

      # Arcaic method :: Checking the priority on the name since each publisher name has a value attached to it.
      if str(self.priority) in msg.pub_name:
        self.get_logger().info('I heard from [%s - %d]: "%s"' % (msg.pub_name, msg.pub_id, msg.data))

    def timer_callback(self):

      if self.priority == self.nmr_pubs:
        self.priority = 1
      else:
        self.priority += 1


def main(args=None):
    rclpy.init(args=args)

    topic = 'topic'
    # check how many publishers there are
    cmd = f'ros2 topic info /{topic}'
    arr = os.popen(cmd).readlines()
    nmr_pub = int(re.split(':', arr[1])[1].strip())

    # Priority will at maximum the same as the nmr of pubs
    # Timers should be treated in a different way since you may want to assign different timers
    minimal_subscriber = MinimalSubscriber(topic, nmr_pub)

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()