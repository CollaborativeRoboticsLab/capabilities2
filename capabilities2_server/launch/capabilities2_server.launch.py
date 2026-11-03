'''
capabilities2_server launch file
'''

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for capabilities2 server

    Returns:
        LaunchDescription: The launch description for capabilities2 server
    """
    # load config file
    server_config = os.path.join(
        get_package_share_directory('capabilities2_server'),
        'config',
        'capabilities.yaml'
    )

    # create cap node
    capabilities2 = Node(
        package='capabilities2_server',
        executable='capabilities2_server_node',
        name='capabilities',
        parameters=[LaunchConfiguration('capabilities_config')],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    launch_proxy = Node(
        package='capabilities2_launch_proxy',
        executable='capabilities2_launch_proxy',
        name='capabilities2_launch_proxy',
        parameters=[LaunchConfiguration('capabilities_config')],
        condition=IfCondition(LaunchConfiguration('enable_launch_proxy')),
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # return
    return LaunchDescription([
        DeclareLaunchArgument('enable_launch_proxy', default_value='true'),
        DeclareLaunchArgument('capabilities_config', default_value=server_config),
        capabilities2,
        launch_proxy,
    ])
