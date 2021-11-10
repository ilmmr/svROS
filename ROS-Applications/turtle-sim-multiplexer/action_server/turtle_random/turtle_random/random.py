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

import rclpy, random, os
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from interfaces.msg import PubSub


# turn - rigth/left move - forward/back
global axis_xy
axis_xy = 'dxy'
global move
move = 'fb'
global rotate
rotate = 'nrl'

global commands
commands = {
    'dn': None,
    'dr': [0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    'dl': [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    'xfn': [0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    'xfr': [1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    'xfl': [1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    'xbn': [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'xbr': [-1.0, 0.0, 0.0, 0.0, 0.0, -1.0],
    'xbl': [-1.0, 0.0, 0.0, 0.0, 0.0, 1.0],
    'yfn': [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
    'yfr': [0.0, 1.0, 0.0, 0.0, 0.0, -1.0],
    'yfl': [0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
    'ybn': [0.0, -1.0, 0.0, 0.0, 0.0, 0.0],
    'ybr': [0.0, -1.0, 0.0, 0.0, 0.0, -1.0],
    'ybl': [0.0, -1.0, 0.0, 0.0, 0.0, 1.0]
    }

class RandomController(Node):

    def __init__(self):

        self.name = 'random'
        self.id = os.getpid()
        super().__init__(self.name)

        # create_publisher declares that the node publishes messages of type String (imported from the std_msgs.msg module), over a topic named topic, and that the “queue size” is 10.
        # Queue size is a required QoS (quality of service) setting that limits the amount of queued messages if a subscriber is not receiving them fast enough.
        self.publisher_ = self.create_publisher(Twist, 'random_topic', 10)
        self.a = random.choice(axis_xy)
        self.r = random.choice(rotate)
        self.m = random.choice(move)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # timer_callback creates a message with the counter value appended, and publishes it to the console with get_logger().info.
    def timer_callback(self):

        cmd = Twist()

        if not (self.a == 'd' and self.r == 'n'):
          if not self.a == 'd':
            temp = str(self.a) + str(self.m) + str(self.r)
          else:
            temp = str(self.a) + str(self.r)

          cmd_aux = commands[temp]

          cmd.linear.x  = random.choice(range(1,4)) * cmd_aux[0] if not cmd_aux[0] == 0.0 else 0.0
          cmd.linear.y  = random.choice(range(1,4)) * cmd_aux[1] if not cmd_aux[1] == 0.0 else 0.0
          cmd.linear.z  = random.choice(range(1,4)) * cmd_aux[2] if not cmd_aux[2] == 0.0 else 0.0

          cmd.angular.x = random.choice(range(1,4)) * cmd_aux[3] if not cmd_aux[3] == 0.0 else 0.0
          cmd.angular.y = random.choice(range(1,4)) * cmd_aux[4] if not cmd_aux[4] == 0.0 else 0.0
          cmd.angular.z = random.choice(range(1,4)) * cmd_aux[5] if not cmd_aux[5] == 0.0 else 0.0


          # msg.data = f'Turn {self.t} and Go {self.m}'
          # msg.pub_name = self.name
          # msg.pub_id = self.id

          self.publisher_.publish(cmd)

        # self.get_logger().info('"%s" by %s' % (msg.data, msg.pub_name))
        self.random_turn_move()

    def random_turn_move(self):
        self.a = random.choice(axis_xy)
        self.r = random.choice(rotate)
        self.m = random.choice(move)

# First the rclpy library is initialized, then the node is created, and then it “spins” the node so its callbacks are called.
def main(args=None):
    rclpy.init(args=args)

    random_c = RandomController()

    rclpy.spin(random_c)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    random_c.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
