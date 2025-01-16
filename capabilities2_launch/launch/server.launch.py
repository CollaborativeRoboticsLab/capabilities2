'''
capabilities2_server launch file
'''

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import FindExecutable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for capabilities2 server

    Returns:
        LaunchDescription: The launch description for capabilities2 server
    """

    filePath = os.path.join(get_package_share_directory('capabilities2_launch'), 'launch_server', 'server.py')

    # create launch proxy node
    launch_interface = Node(
        package='capabilities2_launch',
        executable='capabilities2_launch',
        name='capabilities2_launch',
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    server_process = ExecuteProcess(cmd=[[FindExecutable(name='python3'), ' ', filePath]], shell=True, output='screen')

    # return
    return LaunchDescription([
        launch_interface,
        server_process
    ])
