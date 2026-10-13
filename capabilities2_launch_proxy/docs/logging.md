# Logging in capabilities2_launch_proxy

## Logging behavior

There are three relevant logging paths:

1. Launch framework output
2. Child-process log forwarding into action feedback
3. Optional mirroring into ROS logs from the launch server node

### Default behavior

By default:

- the server publishes launch state in action feedback,
- child process logs at `error` level and above are forwarded into feedback,
- `server.launch.py` suppresses child process stdout and stderr on the proxy terminal,
- child process logs are not additionally mirrored through the server node's ROS logger.

This keeps the action feedback useful without duplicating every child log line in the proxy terminal.

### Optional ROS log mirroring

Set the following parameter to enable server-side ROS log mirroring of child output:

```bash
ros2 run capabilities2_launch_proxy capabilities2_launch_proxy \
	--ros-args -p mirror_child_logs_to_rosout:=true
```

You can also control the minimum log level forwarded into feedback:

```bash
ros2 run capabilities2_launch_proxy capabilities2_launch_proxy \
	--ros-args -p feedback_child_log_level:=warn
```

Supported values follow the server's internal thresholds:

- `debug`
- `info`
- `warn`
- `error`
- `fatal`

You can also override child process output handling at load time:

```bash
ros2 run capabilities2_launch_proxy capabilities2_launch_proxy \
	--ros-args -p override_output_method_to_log:=true
```

When `override_output_method_to_log` is true, the proxy temporarily exports
`OVERRIDE_LAUNCH_PROCESS_OUTPUT=log` while loading the target launch description. This lets the proxy
coerce launch actions that consult that environment variable toward launch-managed logging.

To explicitly keep child stdout and stderr off the proxy terminal while preserving action-feedback
log forwarding, use:

```bash
ros2 launch capabilities2_launch_proxy server.launch.py \
	suppress_child_process_output_to_screen:=true
```

For CLI callers, use `ros2 action send_goal --feedback ...` so forwarded child log lines appear in the
calling terminal instead of only inside the action feedback stream.
