from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtle_random',
            namespace='random',
            executable='random',
            name='random',
            remappings=[
                ('/random/random_topic', '/multiplexer/low_topic'),
            ]
        ),
        Node(
            package='turtle_multiplexer',
            executable='multiplexer',
            namespace='multiplexer',
            name='multiplexer',
        ),
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            namespace='turtlesim_node',
            name='turtlesim_node',
            remappings=[
                ('/turtlesim_node/turtle1/cmd_vel', '/multiplexer/main_topic'),
            ]
        ),
        Node(
           package='turtlesim',
           namespace='turtle_teleop_key',
           executable='turtle_teleop_key',
           name='turtle_teleop_key',
           remappings=[
             ('/turtle_teleop_key/turtle1/cmd_vel', '/multiplexer/high_topic'),
           ]
        ),
    ])
