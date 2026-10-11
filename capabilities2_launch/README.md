# capabilities2_launch

`capabilities2_launch` is a ROS 2 package that starts and supervises launch files through a ROS 2 action server.

It is intended to provide the launch-control part of a capabilities-style runtime in ROS 2, where a capability may need to start a launch file, keep it running, stream state back to a client, and stop it through action cancel.

The package is implemented in Python around a persistent `launch.LaunchService` owned by an `rclpy` node.

## What it provides

- A long-lived ROS 2 action server for starting launch files.
- Launch file resolution by either:
	- absolute path,
	- `package_name + launch_file_name`, or
	- shorthand references like `package:file.launch.py` and `package/file.launch.py`.
- Concurrent management of different launch files.
- Per-launch process tracking and cancellation.
- Action feedback with runtime state and selected child-process log lines.
- Optional mirroring of child-process logs to ROS logging output.

## Current interface

The server exposes one action endpoint relative to the node namespace:

- `~/launch`

When the node name is the default `capabilities2_launch`, the fully qualified action name is:

- `/capabilities2_launch/launch`

## Launch action contract

The package uses `capabilities2_msgs/action/Launch`.

### Goal

You can identify the launch file in one of two ways:

1. Set `package_name` and `launch_file_name`
2. Set `launch_file_path`

Current goal fields:

```text
std_msgs/Header header
string launch_file_path
string package_name
string launch_file_name
```

### Result

```text
std_msgs/Header header
string launch_id
string final_state
string resolved_launch_file_path
string message
```

Important result meanings:

- `launch_id`: currently the canonical resolved launch path used internally as the launch key.
- `final_state`: typically `canceled`, `stopped`, `failed`, or `rejected`.
- `message`: summary of the terminal launch outcome.

### Feedback

```text
std_msgs/Header header
string launch_id
string state
string resolved_launch_file_path
uint32 active_process_count
string message
string log_level
string log_source
string log_text
```

Important feedback behavior:

- `state` reflects the current lifecycle state for the launch, such as `starting`, `running`, or `stopping`.
- `active_process_count` is the current number of tracked child processes still alive.
- `log_level`, `log_source`, and `log_text` are used for child-process log forwarding through feedback.
- By default, only child logs at `error` level and above are forwarded into action feedback.

## How it works

The server owns a single persistent `LaunchService` instance and includes new launch descriptions into that service as action goals arrive.

For each included launch description, the server:

- discovers `ExecuteProcess` actions,
- tracks process start and exit events,
- forwards selected child-process logs into action feedback,
- records process IDs for stop requests,
- cancels the launch by emitting `ShutdownProcess` events when an action goal is canceled.

Each active launch is tracked by canonical file path, so the same launch file cannot be started twice simultaneously. Different launch files can run at the same time.

## Running the server

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run capabilities2_launch capabilities2_launch
```

You can also start it through the package launch file:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch capabilities2_launch server.launch.py
```

## Sending launch goals

### Start by package and launch file name

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	/capabilities2_launch/launch \
	capabilities2_msgs/action/Launch \
	"{package_name: capabilities2_launch, launch_file_name: package_resolved_test.launch.py}"
```

### Start by shorthand package reference

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	/capabilities2_launch/launch \
	capabilities2_msgs/action/Launch \
	"{launch_file_path: capabilities2_launch:package_resolved_test.launch.py}"
```

### Start by absolute path

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	/capabilities2_launch/launch \
	capabilities2_msgs/action/Launch \
	"{launch_file_path: /absolute/path/to/file.launch.py}"
```

## Cancellation model

This package is action-first. To stop a running launch, cancel the corresponding action goal.

The server accepts action cancellation and then:

- marks the launch as stopping,
- emits `ShutdownProcess` events for tracked child processes,
- returns a canceled result to the action client.

## Logging behavior

There are three relevant logging paths:

1. Launch framework output
2. Child-process log forwarding into action feedback
3. Optional mirroring into ROS logs from the launch server node

### Default behavior

By default:

- the server publishes launch state in action feedback,
- child process logs at `error` level and above are forwarded into feedback,
- child process logs are not additionally mirrored through the server node's ROS logger.

This keeps the action feedback useful without duplicating every child log line in the server node output.

### Optional ROS log mirroring

Set the following parameter to enable server-side ROS log mirroring of child output:

```bash
ros2 run capabilities2_launch capabilities2_launch \
	--ros-args -p mirror_child_logs_to_rosout:=true
```

You can also control the minimum log level forwarded into feedback:

```bash
ros2 run capabilities2_launch capabilities2_launch \
	--ros-args -p feedback_child_log_level:=warn
```

Supported values follow the server's internal thresholds:

- `debug`
- `info`
- `warn`
- `error`
- `fatal`

## Example with TurtleBot4 simulation

If the workspace has `turtlebot4_maze_sim` from [CollaborativeRoboticsLab/turtlebot4_simulations](https://github.com/CollaborativeRoboticsLab/turtlebot4_simulations) built and sourced, you can start simulation-related launches through the action server. Otherwise clone and build that repository first.

### Start the simulation world

```bash
source install/setup.bash
ros2 action send_goal \
	/capabilities2_launch/launch \
	capabilities2_msgs/action/Launch \
	"{package_name: turtlebot4_maze_sim, launch_file_name: turtlebot4_maze_small.launch.py}"
```

### Start the navigation stack

```bash
source install/setup.bash
ros2 action send_goal \
	/capabilities2_launch/launch \
	capabilities2_msgs/action/Launch \
	"{package_name: turtlebot4_maze_sim, launch_file_name: sim_navigation.launch.py}"
```

## Parameters

The server currently declares these parameters:

- `mirror_child_logs_to_rosout`:
	- type: `bool`
	- default: `false`
	- controls whether child-process output is mirrored into ROS logs from the server node

- `feedback_child_log_level`:
	- type: `string`
	- default: `error`
	- controls which child log levels are forwarded into action feedback

## Tested behavior

The package currently has integration coverage for:

- package-based launch resolution
- shorthand path resolution
- launched node visibility to external ROS nodes
- action cancellation
- error-log forwarding into action feedback
- optional ROS log mirroring

The focused integration test is in:

- `test/test_capabilities2_launch_integration.py`

## Current limitations

### Launch arguments are not yet supported in the action goal

The current action can identify which launch file to run, but it does not yet carry launch arguments such as:

- `gui:=false`
- `robot_name:=tb4`

For now defined these parameters as launch arguments and set the default values accordingly. If launch-argument support is needed, the action definition and server-side parsing need to be extended.

### Duplicate launch prevention is path-based

The same launch file cannot be active twice at the same time because active launches are keyed by canonical launch path.

### Result status depends on action lifecycle

The action remains active for the lifetime of the launched system. This is intentional for lifecycle ownership, but it means `ros2 action send_goal` behaves like a long-running session unless the goal is canceled or the launch exits.