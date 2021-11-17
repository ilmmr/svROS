from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='py_pubsub',
            namespace='talker1',
            executable='talker1',
            name='talker1',
            remappings=[
                ('/talker1/topic_1', '/multiplexer/high_topic'),
            ]
        ),
        Node(
            package='py_pubsub',
            namespace='talker2',
            executable='talker2',
            name='talker2',
            remappings=[
                ('/talker2/topic_2', '/multiplexer/low_topic'),
            ]
        ),
        Node(
            package='py_pubsub',
            executable='multiplexer',
            namespace='multiplexer',
            name='multiplexer',
        ),
        Node(
            package='py_pubsub',
            executable='subscriber',
            namespace='subscriber',
            name='subscriber',
            remappings=[
                ('/subscriber/main_topic', '/multiplexer/main_topic'),
            ]
        )
    ])
