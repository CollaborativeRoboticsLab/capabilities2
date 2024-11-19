'''
capabilities2_server launch file
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for capabilities2 server

    Returns:
        LaunchDescription: The launch description for capabilities2 server
    """
    # load config file
    server_config = os.path.join(get_package_share_directory('capabilities2_server'), 'config', 'capabilities.yaml')

    # create bridge composition
    capabilities = ComposableNodeContainer(
        name='capabilities2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='capabilities2_server',
                plugin='capabilities2_server::CapabilitiesServer',
                name='capabilities',
                parameters=[server_config]
            )
        ]
    )

    # create launch proxy node
    launch_proxy = Node(
        package='capabilities2_launch',
        executable='capabilities2_launch',
        name='capabilities2_launch'
    )

    # return
    return LaunchDescription([
        capabilities,
        launch_proxy
    ])
