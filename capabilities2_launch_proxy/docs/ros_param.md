# ROS parameters

The server currently declares these parameters:
The combined launch reads them from `launch_proxy` in `capabilities2_server/config/capabilities.yaml`.

- `mirror_child_logs_to_rosout`:
	- type: `bool`
	- default: `false`
	- controls whether child-process output is mirrored into ROS logs from the server node

- `feedback_child_log_level`:
	- type: `string`
	- default: `error`
	- controls which child log levels are forwarded into action feedback

- `override_output_method_to_log`:
	- type: `bool`
	- default: `true` in the shared configuration
	- when `true`, child launch descriptions are loaded with `OVERRIDE_LAUNCH_PROCESS_OUTPUT=log`
	- when `false`, the proxy respects each child launch file's own output setting

- `suppress_child_process_output_to_screen`:
	- type: `bool`
	- default: `true` in the shared configuration
	- rewrites child `Node` and `ExecuteProcess` actions to log-to-file only instead of writing stdout and stderr into the proxy terminal
	- child-process log lines can still be forwarded through action feedback according to `feedback_child_log_level`

- `expand_opaque_functions_for_tracking`:
	- type: `bool`
	- default: `true`
	- expands opaque functions when discovering processes for tracking

- `stop_wait_timeout_sec`:
	- type: `double`
	- default: `10.0`
	- maximum time in seconds to wait for a launch to stop