from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory('capabilities2_launch')
    talker_script = os.path.join(share_dir, 'launch_server', 'test_talker.py')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', talker_script],
            output='screen',
        ),
    ])