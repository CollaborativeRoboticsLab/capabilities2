# Capabilities2 msgs

![ROS](https://img.shields.io/badge/Framework-ROS-informational?&logo=ROS)

This package contains [ROS2](https://index.ros.org/doc/ros2/) `messages`, `services`, and `actions` for the capabilities2 framework.

## Message Types

| Message Type | Description |
| --- | --- |
| `Capability.msg` | A message type for a capability of a robot. |
| `CapabilityCommand.msg` | A message type for a command to a robot. |
| `CapabilityConnection.msg` | A message type for connecting one capability to another through the event system. |
| `CapabilityEvent.msg` | A message type for an event related to a capability. |
| `CapabilityEventCode.msg` | event types |
| `CapabilityEventStamped.msg` | A stamped version of event |
| `CapabilityResponse.msg` | A message type for a response from a robot related to a capability. |
| `CapabilitySpec.msg` | A message type for the specification of a capability. |
| `LaunchParameter.msg` | A key/value launch argument entry used in launch action goals. |
| `LaunchResource.msg` | A per-process resource snapshot for a running launch. |
| `LaunchResourceList.msg` | A server-wide resource snapshot containing aggregate totals and per-process entries. |
| `NaturalCapability.msg` | A message type for a natural capability of a robot. |
| `Remapping.msg` | A message type for a key/value remapping of a capability resource. |
| `RunningCapability.msg` | A message type for a running capability. |

## Service Types

| Service Type | Description |
| --- | --- |
| `EstablishBond.srv` | A service type for establishing a bond between a robot and a client. |
| `FreeCapability.srv` | A service type for freeing a capability of a robot. |
| `GetCapabilitySpec.srv` | A service type for getting the specification of a capability of a robot. |
| `GetCapabilitySpecs.srv` | A service type for getting the specifications of all capabilities of a robot. |
| `GetInterfaces.srv` | A service type for getting the interfaces of a robot. |
| `GetProviders.srv` | A service type for getting the providers of a capability of a robot. |
| `GetRemappings.srv` | A service type for getting the remappings of a capability of a robot. |
| `GetRunningCapabilities.srv` | A service type for getting the running capabilities of a robot. |
| `GetSemanticInterfaces.srv` | A service type for getting the semantic interfaces. |
| `RegisterCapability.srv` | A service type for registering a capability with the server. |
| `StartCapability.srv` | A service type for starting a capability. |
| `StopCapability.srv` | A service type for stopping a capability. |
| `UseCapability.srv` | A service type for using a capability. |

New in 0.1.3:

- `ConnectCapability.srv` - A service type for connecting two running capabilities together through the event system.
- `Launch.srv` - A service type for starting or stopping a launch by `package_name + launch_file_name` or `launch_file_path`, with optional `reference_name` and `launch_parameters` on start.
- `TriggerCapability.srv` - A service type for triggering a capability.

## Action Types

| Action Type | Description |
| --- | --- |
| `Capability.action` | An action type for an encapsulated capability action. |
| `Launch.action` | An action type for launching a launch file. |

## Launch-related interface summary

The launch-control path now uses these additional launch-related contracts:

- `LaunchParameter.msg`:
	- `key`
	- `value`
- `LaunchResource.msg`:
	- identifies one tracked process by key, launch reference, process name, and PID
	- carries current uptime, CPU, RSS, and VMS values for that process
- `LaunchResourceList.msg`:
	- carries one server-wide snapshot of all active tracked launch processes
	- includes aggregate counts and total CPU/RAM across the currently active launches

`Launch.action` now supports:

- `reference_name` for caller-defined grouping and resource labeling
- `launch_parameters` as a list of `LaunchParameter.msg` entries
- resource-oriented feedback fields such as main and child PID lists plus a nested `LaunchResource.msg` snapshot for current launch-level totals

These launch resource messages are snapshot-oriented, not historical summaries. Memory fields represent the current sampled values, and total counts reflect only the launches and processes active at the time the snapshot was published.
