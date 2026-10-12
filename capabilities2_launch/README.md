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
- Per-goal launch arguments passed as key/value pairs.
- Concurrent management of different launch files.
- Per-launch process tracking and cancellation.
- Action feedback with runtime state, selected child-process log lines, and per-launch resource totals.
- A server-wide resource topic with aggregate totals and per-process resource entries across all active launches.
- Optional mirroring of child-process logs to ROS logging output.

## Current interfaces

The server exposes one action endpoint relative to the node namespace:

- `~/launch`

It also exposes one resource topic relative to the node namespace:

- `~/resources`

When the node name is the default `capabilities2_launch`, the fully qualified action name is:

- `/capabilities2_launch/launch`

and the resource topic is:

- `/capabilities2_launch/resources`

## Launch action contract

The package uses `capabilities2_msgs/action/Launch`.

### Goal

You can identify the launch file in one of two ways:

1. Set `package_name` and `launch_file_name`
2. Set `launch_file_path`

Current goal fields:

```text
std_msgs/Header header
string reference_name
string launch_file_path
string package_name
string launch_file_name
capabilities2_msgs/LaunchParameter[] launch_parameters
```

Important goal behavior:

- `reference_name` is a caller-defined label used in resource reporting and process keys.
- `launch_parameters` carries launch arguments as key/value pairs such as `gui=false` or `robot_name=tb4`.
- Each `launch_parameters` entry is applied as a launch configuration before the target launch description runs.

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
uint32 child_process_count
int64[] main_pids
int64[] child_pids
float32 launch_uptime_sec
float32 cpu_usage_percent
float32 peak_sampled_cpu_usage_percent
float32 time_above_80pct_peak_sampled_cpu_sec
uint64 memory_rss_bytes
uint64 peak_memory_rss_bytes
float32 time_above_90pct_peak_rss_sec
uint64 memory_vms_bytes
uint64 peak_memory_vms_bytes
float32 time_above_90pct_peak_vms_sec
string message
string log_level
string log_source
string log_text
```

Important feedback behavior:

- `state` reflects the current lifecycle state for the launch, such as `starting`, `running`, or `stopping`.
- `active_process_count` is the current number of tracked root launch processes still alive.
- `child_process_count` is the current number of discovered descendant processes under those tracked roots.
- `main_pids` and `child_pids` identify the currently sampled process tree for that launch.
- `cpu_usage_percent`, `memory_rss_bytes`, and `memory_vms_bytes` are current per-launch totals across the sampled process tree.
- `peak_sampled_cpu_usage_percent` is the retained maximum sampled interval CPU value seen for that launch during its current lifetime.
- `peak_memory_rss_bytes` and `peak_memory_vms_bytes` are retained peak aggregate memory values for that launch during its current lifetime.
- `time_above_80pct_peak_sampled_cpu_sec`, `time_above_90pct_peak_rss_sec`, and `time_above_90pct_peak_vms_sec` are cumulative durations gathered during the current launch lifetime.
- `log_level`, `log_source`, and `log_text` are used for child-process log forwarding through feedback.
- By default, only child logs at `error` level and above are forwarded into action feedback.

## Resource topic contract

The package publishes `capabilities2_msgs/msg/LaunchResourceList` on `~/resources`.

Each message contains:

- aggregate counts across all currently active launches
- aggregate CPU and memory totals across all currently active tracked processes
- retained peak CPU and memory totals across the currently active launches
- cumulative threshold-duration counters across the currently active launches
- a list of `LaunchResource` entries, one per currently sampled process

Each `LaunchResource` entry contains:

- a stable process key built from `reference_name`, process name, and PID
- launch identity fields such as `reference_name`, `launch_id`, state, and resolved launch path
- process identity fields such as `process_name`, `pid`, `parent_pid`, and `is_main_process`
- current sampled resource fields such as process uptime, CPU estimate, RSS, and VMS
- retained peak sampled CPU and retained peak memory values for that process during the current launch lifetime
- cumulative threshold-duration counters for that process during the current launch lifetime

### Snapshot semantics

The resource data is snapshot-oriented.

- Counts, PID lists, and memory fields represent the current sampled state of the active process tree.
- `launch_uptime_sec` and `process_uptime_sec` are current point-in-time uptime values, not accumulated durations over past runs.
- `cpu_usage_percent` is not a lifetime average. It is computed from the delta between the two most recent samples for each active process and then summed for the launch or server-wide snapshot.
- `peak_sampled_cpu_usage_percent` is the highest observed sampled interval CPU value so far, not a rolling or lifetime average.
- `peak_memory_rss_bytes` and `peak_memory_vms_bytes` are retained maxima seen so far during the current launch lifetime.
- `time_above_80pct_peak_sampled_cpu_sec` accumulates sampled elapsed time while the current sampled CPU remains at or above `80%` of the retained peak sampled CPU.
- `time_above_90pct_peak_rss_sec` and `time_above_90pct_peak_vms_sec` accumulate sampled elapsed time while current memory remains at or above `90%` of the retained peak memory values.
- Short-lived helper processes may appear only if they are alive when a sample is taken.
- The server does not currently retain historical samples, rolling averages, or post-exit resource history.

## How it works

The server owns a single persistent `LaunchService` instance and includes new launch descriptions into that service as action goals arrive.

The active implementation is split across:

- `capabilities2_launch.py` for the ROS node and action server surface
- `launch_runtime.py` for launch inclusion, process events, child log forwarding, and resource publication
- `resource_tracker.py` for `/proc` sampling and peak/threshold accounting

For each included launch description, the server:

- discovers `ExecuteProcess` actions,
- applies caller-provided launch configurations before the description runs,
- tracks process start and exit events,
- forwards selected child-process logs into action feedback,
- records process IDs for stop requests,
- samples `/proc` for the active process tree,
- publishes a server-wide resource snapshot topic,
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
	"{reference_name: package-test, package_name: capabilities2_launch, launch_file_name: package_resolved_test.launch.py}"
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

### Start with launch parameters

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	/capabilities2_launch/launch \
	capabilities2_msgs/action/Launch \
	"{
	  reference_name: tb4-maze,
	  package_name: turtlebot4_maze_sim,
	  launch_file_name: turtlebot4_maze_small.launch.py,
	  launch_parameters: [
	    {key: gui, value: 'false'},
	    {key: robot_name, value: turtlebot4}
	  ]
	}"
```

## Reading resource data

To inspect the live server-wide resource snapshot:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /capabilities2_launch/resources
```

The topic contains one snapshot per publish interval, not an accumulated history.

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
	"{
	  reference_name: tb4-maze,
	  package_name: turtlebot4_maze_sim,
	  launch_file_name: turtlebot4_maze_small.launch.py,
	  launch_parameters: [
	    {key: gui, value: 'false'}
	  ]
	}"
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
- launch parameter passing into a launch file
- launched node visibility to external ROS nodes
- action cancellation
- error-log forwarding into action feedback
- optional ROS log mirroring
- server-wide resource topic publication
- child-process resource discovery

The focused integration test is in:

- `test/test_capabilities2_launch_integration.py`

## Current limitations

### Duplicate launch prevention is path-based

The same launch file cannot be active twice at the same time because active launches are keyed by canonical launch path.

### Resource reporting is current-state only

Resource reporting is designed for live observability, not historical accounting.

- CPU is based on recent sample deltas, not lifetime averages.
- Peak CPU and peak memory are retained only for the current lifetime of the active launch.
- Memory includes both current sampled values and retained peak sampled values, but not post-exit history or system-wide GPU/VRAM statistics.
- Exited processes are removed from subsequent snapshots.

### Result status depends on action lifecycle

The action remains active for the lifetime of the launched system. This is intentional for lifecycle ownership, but it means `ros2 action send_goal` behaves like a long-running session unless the goal is canceled or the launch exits.