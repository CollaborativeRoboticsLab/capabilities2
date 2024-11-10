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
    executor_config = os.path.join(get_package_share_directory('capabilities2_executor'), 'config', 'executor.yaml')

    executor = Node(
            package='capabilities2_executor',
            namespace='',
            executable='capabilities2_executor',
            name='capabilities2_executor'
        )
    
    executor_file = Node(
            package='capabilities2_executor',
            namespace='',
            executable='capabilities2_executor_file',
            name='capabilities2_executor_file',
            parameters=[executor_config]
        )

    # return
    return LaunchDescription([
        executor,
        executor_file
    ])
