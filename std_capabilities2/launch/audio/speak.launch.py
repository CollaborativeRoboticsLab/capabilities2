from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    speak_launch_path = os.path.join(get_package_share_directory('hri_audio_stack'), 'launch', 'speak.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(speak_launch_path),
        )
    ])