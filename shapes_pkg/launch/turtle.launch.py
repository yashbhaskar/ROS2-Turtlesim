from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            namespace='turtlesim1',
            package='turtlesim',
            executable='turtlesim_node',
            output='screen'
        ),
        Node(
            package='shapes_pkg',
            executable='teleport',
            name='sim',
            output='screen',
            remappings=[('/turtle1/teleport_absolute', '/turtlesim1/turtle1/teleport_absolute')]

        ),
    ])
