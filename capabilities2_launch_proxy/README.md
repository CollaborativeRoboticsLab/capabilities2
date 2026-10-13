# Capabilities2 Launch Proxy

`capabilities2_launch_proxy` is a ROS 2 package that starts and supervises launch files through a ROS 2 action server, with a simple start/stop service surface retained for compatibility.

It is intended to provide the launch-control part of a capabilities-style runtime in ROS 2, where a capability may need to start a launch file, keep it running, stream state back to a client, and stop it through action cancel.

The package is implemented in Python around a persistent `launch.LaunchService` owned by an `rclpy` node.

## What it provides

- A long-lived ROS 2 action server for starting launch files.
- A compatibility launch service for start requests plus a stop service for stopping a running launch by package/file reference or direct launch reference.
- Launch file resolution by either:
	- absolute path,
	- `package_name + launch_file_name`, or
	- shorthand references like `package:file.launch.py` and `package/file.launch.py`.
- Per-goal launch arguments passed as key/value pairs.
- Concurrent management of different launch files.
- Per-launch process tracking and cancellation.
- Action feedback with runtime state, selected child-process log lines, and per-launch resource totals.
- A server-wide resource topic with aggregate totals and per-process resource entries across all active launches.
- Optional mirroring of child-process logs to ROS logging output.
- Optional suppression of child-process stdout and stderr on the proxy terminal while still forwarding selected log lines through action feedback.

## How it works

### Running the server

You can also start it through the package launch file:

```bash
source install/setup.bash
ros2 launch capabilities2_launch_proxy server.launch.py
```

### Sending launch action goals

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	"{reference_name: package-test, package_name: capabilities2_launch_proxy, launch_file_name: package_resolved_test.launch.py}"
```

### Starting and stopping by service

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/srv/Launch \
	"{package_name: capabilities2_launch_proxy, launch_file_name: package_resolved_test.launch.py}"

ros2 service call \
	/capabilities2_launch_proxy/stop_launch \
	capabilities2_msgs/srv/Launch \
	"{package_name: capabilities2_launch_proxy, launch_file_name: package_resolved_test.launch.py}"
```

### Inspect the live server-wide resource snapshot:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /capabilities2_launch_proxy/resources
```
The topic contains one snapshot per publish interval, not an accumulated history.

## Cancellation model

This package is action-first. To stop a running launch, cancel the corresponding action goal.

For compatibility-oriented callers that need simple start/stop behavior, the server also exposes `Launch.srv` endpoints on `~/launch` and `~/stop_launch`. That service now supports either `package_name + launch_file_name` or `launch_file_path`, and start requests may also include `launch_parameters`.

Service-based start requests may also set `reference_name` so resource snapshots use a stable caller-defined label instead of falling back to the launch key.

The server accepts action cancellation and then:

- marks the launch as stopping,
- emits `ShutdownProcess` events for tracked child processes,
- returns a canceled result to the action client.

## Read more information

- [ROS interface](./docs/ros_interface.md) for the action server, compatibility service endpoints, and resource topic.
- [Usage](./docs/usage.md) for running the server and sending launch goals.
- [Logging](./docs/logging.md) for child-process log forwarding and optional ROS log mirroring.
- [Example use case](./docs/example_usecase.md) for a TurtleBot4 simulation launch through the action server.
- [ROS parameters](./docs/ros_param.md) for the information and [configuration](./config/launch_proxy.yaml) or actual launch file overrides.
- [Limitations](./docs/limitations.md) for current known limitations of the package.
- [Testing](./docs/testing.md) for the included test suite and how to run it.