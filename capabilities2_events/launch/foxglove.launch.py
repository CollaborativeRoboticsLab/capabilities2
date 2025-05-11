from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # Declare launch arguments
    launch_args = [
        DeclareLaunchArgument('port', default_value='8765'),
        DeclareLaunchArgument('address', default_value='0.0.0.0'),
        DeclareLaunchArgument('tls', default_value='false'),
        DeclareLaunchArgument('certfile', default_value=''),
        DeclareLaunchArgument('keyfile', default_value=''),
        DeclareLaunchArgument('topic_whitelist', default_value="['/events']"),
        # DeclareLaunchArgument('topic_whitelist', default_value="['.*']"),  # Uncomment this line to whitelist all topics
        DeclareLaunchArgument('param_whitelist', default_value="['.*']"),
        DeclareLaunchArgument('service_whitelist', default_value="['.*']"),
        DeclareLaunchArgument('client_topic_whitelist', default_value="['.*']"),
        DeclareLaunchArgument('min_qos_depth', default_value='1'),
        DeclareLaunchArgument('max_qos_depth', default_value='10'),
        DeclareLaunchArgument('num_threads', default_value='0'),
        DeclareLaunchArgument('send_buffer_limit', default_value='10000000'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('capabilities', default_value="[clientPublish,parameters,parametersSubscribe,services,connectionGraph,assets]"),
        DeclareLaunchArgument('include_hidden', default_value='false'),
        DeclareLaunchArgument(
            'asset_uri_allowlist',
            default_value=r"['^package://(?:[-\\w]+/)*[-\\w]+\\.(?:dae|fbx|glb|gltf|jpeg|jpg|mtl|obj|png|stl|tif|tiff|urdf|webp|xacro)$']"
        ),
        DeclareLaunchArgument('ignore_unresponsive_param_nodes', default_value='true'),
    ]

    # Define the Node
    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'address': LaunchConfiguration('address'),
            'tls': LaunchConfiguration('tls'),
            'certfile': LaunchConfiguration('certfile'),
            'keyfile': LaunchConfiguration('keyfile'),
            'topic_whitelist': LaunchConfiguration('topic_whitelist'),
            'param_whitelist': LaunchConfiguration('param_whitelist'),
            'service_whitelist': LaunchConfiguration('service_whitelist'),
            'client_topic_whitelist': LaunchConfiguration('client_topic_whitelist'),
            'min_qos_depth': LaunchConfiguration('min_qos_depth'),
            'max_qos_depth': LaunchConfiguration('max_qos_depth'),
            'num_threads': LaunchConfiguration('num_threads'),
            'send_buffer_limit': LaunchConfiguration('send_buffer_limit'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'capabilities': LaunchConfiguration('capabilities'),
            'include_hidden': LaunchConfiguration('include_hidden'),
            'asset_uri_allowlist': LaunchConfiguration('asset_uri_allowlist'),
            'ignore_unresponsive_param_nodes': LaunchConfiguration('ignore_unresponsive_param_nodes'),
        }],
        output='screen'
    )

    return LaunchDescription(launch_args + [foxglove_bridge_node])