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

class Multiplexer(Node):

    def __init__(self):
        super().__init__('multiplexer_controller')
        self.timer_major = 10

        self.priority = True # can also start False, but random will take control firstly
        self.nmr_pubs = 2
        self.timer = None

        # define 2 topics subscriber
        self.low_topic = 'low_topic'
        self.high_topic = 'high_topic'


        self.publisher_ = self.create_publisher(PubSub, 'main_topic', 10)

        # create a subscription for each publisher
        # for i in range(nmr):
          # self.init_subscription(self.timer_n, topics_info[i+1])
        self.init_subscription(self.timer_major, self.high_topic, 1)
        self.init_subscription(5, self.low_topic, 2)

    def init_subscription(self, timer, topic, priority):

      self.subscription = self.create_subscription(
            PubSub,
            topic,
            self.high_callback if priority == 1 else self.low_callback, 10)
            # lambda msg: self.publisher_.publish(msg) if self.priority == priority else False

        #if self.priority == priority:
        #self.timer = self.create_timer(timer, self.timer_callback)

      self.subscription

      # The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.
      # The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.


    def high_callback(self, msg):

        self.priority = True
        msg.data = f'{msg.data}, with timer set to {self.timer_major}'
        self.publisher_.publish(msg)

        # the timer must be reset, otherwise after the 10s it will not be considered as a new timer.
        if self.timer:
          self.destroy_timer(self.timer)

        self.timer = self.create_timer(self.timer_major, self.timer_callback)


    def low_callback(self, msg):

        if self.priority == False:
          self.publisher_.publish(msg)


    def timer_callback(self):
      self.priority = False
      # if self.priority == self.nmr_pubs:
        # self.priority = 1
      # else:
        # self.priority += 1


def main(args=None):
    rclpy.init(args=args)

    '''
    other version

      topic = 'topic'
      # check how many publishers there are related to the topic
      cmd_topic_info = f'ros2 topic info /{topic}'
      topic_info = os.popen(cmd_topic_info).readlines()
      nmr_pub = int(re.split(':', topic_info[1])[1].strip())

      # check who are the nodes related to the topic
      publishers = []
      cmd_node_list = f'ros2 node list'
      node_list = os.popen(cmd_node_list).readlines()
      for node in node_list:
        node = node.strip()
        cmd_node_info = f'ros2 node info {node}'
        node_info = os.popen(cmd_node_info).read()
        publisher_info = re.search(r'Publishers:\n(.|\n)*(?=Service Servers)', node_info).group(0)

        # if topic in the info of the publisher
        if f'/{topic}:' in publisher_info:
          publishers.append(node[1:])

      # print(publishers) - Debug

      # Priority will at maximum the same as the nmr of pubs
      # Timers should be treated in a different way since you may want to assign different timers
      minimal_subscriber = MinimalSubscriber(topic, nmr_pub, publishers)
    '''

    multiplexer = Multiplexer()

    rclpy.spin(multiplexer)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    multiplexer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
