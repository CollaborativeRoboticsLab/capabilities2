'''
capabilities2_server launch file
'''

import os

import yaml

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

    with open(server_config, encoding='utf-8') as config:
        proxy_parameters = yaml.safe_load(config)['/**']['ros__parameters']['launch_proxy']

    # create cap node
    capabilities2 = Node(
        package='capabilities2_server',
        executable='capabilities2_server_node',
        name='capabilities',
        parameters=[server_config],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    launch_proxy = Node(
        package='capabilities2_launch_proxy',
        executable='capabilities2_launch_proxy',
        name='capabilities2_launch_proxy',
        parameters=[proxy_parameters],
        condition=IfCondition(LaunchConfiguration('enable_launch_proxy')),
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    # return
    return LaunchDescription([
        DeclareLaunchArgument('enable_launch_proxy', default_value='true'),
        capabilities2,
        launch_proxy,
    ])
