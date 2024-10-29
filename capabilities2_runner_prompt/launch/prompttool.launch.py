from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    prompt_launch_path = os.path.join(get_package_share_directory('prompt_bridge'), 'launch', 'prompt_bridge.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(prompt_launch_path),
        )
    ])