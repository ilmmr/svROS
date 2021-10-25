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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

# Using the same subscriber node, that keeps getting destroyed.
class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.priority = 1

        # init first subscription
        self.init_subscription(6, 'topic_1')

    def init_subscription(self, timer, topic_name):

      self.subscription = self.create_subscription(
            String,
            topic_name,
            lambda msg: self.get_logger().info('I heard: "%s"' % msg.data), 10)

      self.timer = self.create_timer(timer, self.timer_callback)
      self.subscription

      # The subscriber node’s code is nearly identical to the publisher’s. The constructor creates a subscriber with the same arguments as the publisher. Recall from the topics tutorial that the topic name and message type used by the publisher and subscriber must match to allow them to communicate.
      # The subscriber’s constructor and callback don’t include any timer definition, because it doesn’t need one. Its callback gets called as soon as it receives a message.


    def listener_callback(self, msg):
      self.get_logger().info('I heard: "%s"' % msg.data)


    def timer_callback(self):

      # Destroy the current subscription
      self.destroy_all()

      if self.priority == 0:
        self.init_subscription(6, 'topic_1')
        self.priority = 1
      else:
        self.init_subscription(3, 'topic_2')
        self.priority = 0

    def destroy_all(self):
      self.destroy_subscription(self.subscription)
      self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
