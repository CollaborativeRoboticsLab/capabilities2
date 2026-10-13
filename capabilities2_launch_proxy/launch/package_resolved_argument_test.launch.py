from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    share_dir = get_package_share_directory('capabilities2_launch_proxy')
    talker_script = os.path.join(share_dir, 'test_nodes', 'test_argument_talker.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'test_message',
            default_value='default-launch-message',
            description='Message published by the argument test talker.',
        ),
        ExecuteProcess(
            cmd=['python3', talker_script, LaunchConfiguration('test_message')],
            output='screen',
        ),
    ])