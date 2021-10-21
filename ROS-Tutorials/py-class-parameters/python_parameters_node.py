import rclpy
import rclpy.node
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import ParameterType

class MinimalParam(rclpy.node.Node):
    def __init__(self):
        super().__init__('minimal_param_node')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Timer is initialized (with timer_period set as 2 seconds), which causes the timer_callback function to be executed once every two seconds.
        # The line self.declare_parameter('my_parameter', 'world') of the constructor creates a parameter with the name my_parameter and a default value of world.
        self.declare_parameter('my_parameter', 'world')

    # The first line of our timer_callback function gets the parameter my_parameter from the node, and stores it in my_param.
    # Next,the get_logger function ensures the message is logged.
    # Then, we set the parameter ‘my_parameter’ back to the default string value ‘world’.
    def timer_callback(self):
        my_param = self.get_parameter('my_parameter').get_parameter_value().string_value

        self.get_logger().info('Hello %s!' % my_param)

        my_new_param = rclpy.parameter.Parameter(
            'my_parameter',
            rclpy.Parameter.Type.STRING,
            'world'
        )
        all_new_parameters = [my_new_param]
        self.set_parameters(all_new_parameters)

# Except the first message where the parameter had a default value (an empty string), the terminal should return the following message every 2 seconds:
# -- [INFO] [parameter_node]: Hello world! --
def main():
    rclpy.init()
    node = MinimalParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
