from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        # Note the only difference between the two nodes is their namespace values. Unique namespaces allow the system to start two simulators without node name nor topic name conflicts.
        # Both turtles in this system receive commands over the same topic and publish their pose over the same topic.
        # Without unique namespaces, there would be no way to distinguish between messages meant for one turtle or the other.
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
        # mimic’s /input/pose topic is remapped to /turtlesim1/turtle1/pose and it’s /output/cmd_vel topic to /turtlesim2/turtle1/cmd_vel.
        # This means mimic will subscribe to /turtlesim1/sim’s pose topic and republish it for /turtlesim2/sim’s velocity command topic to subscribe to.
        # In other words, turtlesim2 will mimic turtlesim1’s movements.
    ])
# To see the system in action, open a new terminal and run the ros2 topic pub command on the /turtlesim1/turtle1/cmd_vel topic to get the first turtle moving:
# - ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}" -
