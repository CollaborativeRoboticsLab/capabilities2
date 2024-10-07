# capabilities2_launch_proxy

Provides an Action API proxy to the ros2 launch framework. Implements features that are only available in python since the `launch` API is not available in C++.

Contains the essential features to implement the ros1 [`capabilities`](https://wiki.ros.org/capabilities) package launch functionality in ros2.

Use in conjunction with `capabilities2_server` package, to provide a full capabilities server.

## how it works

Implements three main functions:

| Function | Description |
| --- | --- |
| `launch` | Run a launch file requested a runtime via a network request |
| `shutdown` | shutdown a launch file that has been started |
| `events` | send capability events during operation |

The lifecylce of these functions are typically handled by the capabilities server but could be used for other things.

## Use without capabilities2 server

The proxy provides an action called `~/launch`. The `goal` is a file path to launch from. The feedback provides events about the launch status.

### Add to a launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # create launch proxy node
        Node(
            package='capabilities2_launch_proxy',
            executable='capabilities_launch_proxy',
            name='capabilities_launch_proxy'
        )
    ])
```

### Run standalone

```bash
ros2 run capabilities2_launch_proxy capabilities_launch_proxy
```
