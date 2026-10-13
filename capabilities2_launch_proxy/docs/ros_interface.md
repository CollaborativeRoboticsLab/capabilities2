# ROS Interface

The server exposes one action endpoint relative to the node namespace:

- `~/launch`

It also exposes two compatibility service endpoints relative to the node namespace:

- `~/launch`
- `~/stop_launch`

It also exposes one resource topic relative to the node namespace:

- `~/resources`

When the node name is the default `capabilities2_launch_proxy`, the fully qualified action name is:

- `/capabilities2_launch_proxy/launch`

The fully qualified compatibility service names are:

- `/capabilities2_launch_proxy/launch`
- `/capabilities2_launch_proxy/stop_launch`

and the resource topic is:

- `/capabilities2_launch_proxy/resources`

## Launch compatibility service contract

The compatibility start and stop services both use `capabilities2_msgs/srv/Launch`.

### Request

```text
std_msgs/Header header
string reference_name
string launch_file_path
string package_name
string launch_file_name
capabilities2_msgs/LaunchParameter[] launch_parameters
```

Service behavior:

- when `reference_name` is set, the started launch uses it for resource labeling and process keys instead of falling back to the resolved launch key
- when `launch_file_path` is set, it takes precedence and is resolved using the same shorthand or absolute-path rules as the action interface
- when `package_name` is set, the server resolves `package_name + launch_file_name`
- when `package_name` is empty, `launch_file_name` is treated as a direct launch reference and may be an absolute path or shorthand reference accepted by the runtime
- the start service accepts `launch_parameters` and applies them before the launch description runs
- the compatibility service still does not expose action-style progress feedback; use the action interface when you need lifecycle feedback streaming

### Response

```text
std_msgs/Header header
string status
```

Response meanings:

- start returns values such as `successfully started` or `already started. ignoring`
- stop returns values such as `successfully stopped` or `already stopped. ignoring`
- runtime resolution or inclusion failures are returned as plain-text error status strings

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
capabilities2_msgs/LaunchResource resource
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
- `resource` carries the current per-launch resource snapshot using `LaunchResource.msg`.
- `resource.cpu_usage_percent`, `resource.memory_rss_bytes`, and `resource.memory_vms_bytes` are current per-launch totals across the sampled process tree.
- `resource.peak_sampled_cpu_usage_percent` is the retained maximum sampled interval CPU value seen for that launch during its current lifetime.
- `resource.peak_memory_rss_bytes` and `resource.peak_memory_vms_bytes` are retained peak aggregate memory values for that launch during its current lifetime.
- `resource.time_above_80pct_peak_sampled_cpu_sec`, `resource.time_above_90pct_peak_rss_sec`, and `resource.time_above_90pct_peak_vms_sec` are cumulative durations gathered during the current launch lifetime.
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

- `launch_node.py` for the ROS node and action server surface
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
