# Launch file using event handlers :

from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    spawn_node = Node(
        package='shapes_pkg',
        executable='turtle_spawn',
        name='spawner',
        output='screen'
    )

    random_node = Node(
        package='shapes_pkg',
        executable='random',
        name='random',
        output='screen'
    )

    # Start spawn_node 2 seconds after turtlesim_node starts
    start_spawn = RegisterEventHandler(
        OnProcessStart(
            target_action=turtlesim_node,
            on_start=[
                LogInfo(msg="turtlesim started. Waiting 2s..."),
                TimerAction(period=2.0, actions=[spawn_node])
            ]
        )
    )

    # Start random_node 2 seconds after spawn_node starts
    start_random = RegisterEventHandler(
        OnProcessStart(
            target_action=spawn_node,
            on_start=[
                LogInfo(msg="turtle_spawn started. Waiting 2s..."),
                TimerAction(period=2.0, actions=[random_node])
            ]
        )
    )

    return LaunchDescription([
        turtlesim_node,
        start_spawn,
        start_random
    ])

