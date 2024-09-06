# empty launch file running an empty node

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """
    Generate launch description for empty node

    Returns:
        LaunchDescription: The launch description for empty node
    """
    # empty node container
    node = ComposableNodeContainer(
        name='empty_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # ComposableNode(
            #     # talker node
            #     package='std_capabilities',
            #     plugin='std_capabilities::Empty',
            #     name='empty'
            # )
        ],
        output='screen'
    )

    return LaunchDescription([
        node
    ])
