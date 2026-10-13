import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    share_dir = get_package_share_directory('capabilities2_launch_proxy')
    child_launch = os.path.join(share_dir, 'launch', 'package_resolved_test.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(child_launch)),
    ])