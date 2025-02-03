'''
capabilities2_server launch file
'''

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for capabilities2 server

    Returns:
        LaunchDescription: The launch description for capabilities2 executor
    """
    # load config file
    fabric_config = os.path.join(get_package_share_directory('capabilities2_fabric'), 'config', 'fabric.yaml')

    capabilities2_fabric = Node(
            package='capabilities2_fabric',
            namespace='',
            executable='capabilities2_fabric',
            name='capabilities2_fabric',
            output='screen'
        )
    
    fabric_client = Node(
            package='capabilities2_fabric',
            namespace='',
            executable='fabric_client',
            name='fabric_client',
            parameters=[fabric_config],
            output='screen'
        )

    return LaunchDescription([
        capabilities2_fabric,
        fabric_client
    ])
