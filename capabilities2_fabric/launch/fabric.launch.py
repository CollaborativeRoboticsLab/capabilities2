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
    executor_config = os.path.join(get_package_share_directory('capabilities2_fabric'), 'config', 'executor.yaml')

    executor = Node(
            package='capabilities2_fabric',
            namespace='',
            executable='capabilities2_fabric',
            name='capabilities2_fabric',
            output='screen'
        )
    
    executor_file = Node(
            package='capabilities2_fabric',
            namespace='',
            executable='capabilities2_file_parser',
            name='capabilities2_file_parser',
            parameters=[executor_config],
            output='screen'
        )

    # return
    return LaunchDescription([
        executor,
        executor_file
    ])
