from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='python_parameters',
            executable='param_talker',
            name='custom_parameter_node',

            # -- By adding the two lines below, we ensure our output is printed in our console.
            output='screen',
            emulate_tty=True,
            # --
            parameters=[
                {'my_parameter': 'earth'}
            ]
        )
    ])

