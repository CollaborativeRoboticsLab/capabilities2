'''
capabilities2_server launch file
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for capabilities2 server

    Returns:
        LaunchDescription: The launch description for capabilities2 events listener
    """
    # create bridge composition
    capabilities2 = Node(
        package='capabilities2_events',
        executable='capabilities2_events_node',
        name='listener',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # return
    return LaunchDescription([
        capabilities2
    ])
