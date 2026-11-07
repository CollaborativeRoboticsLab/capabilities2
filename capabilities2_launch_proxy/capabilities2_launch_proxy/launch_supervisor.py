import json
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource


def main():
    launch_file_path = sys.argv[1]
    launch_parameters = json.loads(sys.argv[2])
    ready_token = sys.argv[3]
    launch_service = LaunchService()
    launch_service.include_launch_description(LaunchDescription([
        OpaqueFunction(function=lambda context: print(ready_token, flush=True)),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(launch_file_path),
            launch_arguments=launch_parameters,
        ),
    ]))
    return launch_service.run()


if __name__ == '__main__':
    sys.exit(main())