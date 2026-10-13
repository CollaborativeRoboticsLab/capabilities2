from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory('capabilities2_launch_proxy')
    listener_script = os.path.join(share_dir, 'test_nodes', 'test_listener_echo.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', listener_script],
            output='screen',
        ),
    ])