'''
capabilities2 launch server
'''

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for the unified capabilities2 launch server."""

    config_file = os.path.join(
        get_package_share_directory('capabilities2_launch_proxy'),
        'config',
        'launch_proxy.yaml',
    )

    mirror_child_logs_to_rosout = LaunchConfiguration('mirror_child_logs_to_rosout')
    feedback_child_log_level = LaunchConfiguration('feedback_child_log_level')
    override_output_method_to_log = LaunchConfiguration('override_output_method_to_log')
    suppress_child_process_output_to_screen = LaunchConfiguration('suppress_child_process_output_to_screen')
    expand_opaque_functions_for_tracking = LaunchConfiguration('expand_opaque_functions_for_tracking')

    launch_server = Node(
        package='capabilities2_launch_proxy',
        executable='capabilities2_launch_proxy',
        name='capabilities2_launch_proxy',
        output='screen',
        parameters=[
            config_file,
            {
                'mirror_child_logs_to_rosout': mirror_child_logs_to_rosout,
                'feedback_child_log_level': feedback_child_log_level,
                'override_output_method_to_log': override_output_method_to_log,
                'suppress_child_process_output_to_screen': suppress_child_process_output_to_screen,
                'expand_opaque_functions_for_tracking': expand_opaque_functions_for_tracking,
            },
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mirror_child_logs_to_rosout',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'feedback_child_log_level',
            default_value='error',
        ),
        DeclareLaunchArgument(
            'override_output_method_to_log',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'suppress_child_process_output_to_screen',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'expand_opaque_functions_for_tracking',
            default_value='true',
        ),
        launch_server,
    ])
