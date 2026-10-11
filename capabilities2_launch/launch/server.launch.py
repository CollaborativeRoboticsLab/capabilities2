'''
capabilities2 launch server
'''

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the unified capabilities2 launch server."""

    launch_server = Node(
        package='capabilities2_launch',
        executable='capabilities2_launch',
        name='capabilities2_launch',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        launch_server,
    ])
