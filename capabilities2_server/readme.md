# Capabilities2 Server

Capabilities2 server allows interaction with the capabilities2 API. It is a reimplementation of the capabilities package using C++ (original capabilities package was written in Python 2.7). In addition to C++ reimplementation being more resource efficient, this package extends the capabilities package features in following ways,

1. Database support for storing capabilities.
2. Capability registration through service calls.
3. Abstracted providers through a *runners* API see [capabilities2_runner](../capabilities2_runner/readme.md)

## Specialising the capabilities2 server

The capabilities2 server is implemented as a [ROS2 Component](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html). The server can be specialised by composing it with another Component that adds domain logic.

## Launch capabilities2 server

```bash
source install/setup.bash
ros2 launch capabilities2_server server.launch.py
```

Refer `capabilities2_server/config/capabilities.yaml` for launch arguments.

## System dependencies

The capabilities2 server depends on the following `bondcpp` ROS2 package. See the package.xml for the full list of dependencies. Notably, the capabilities2 server depends on the following system dependencies:

- `sqlite3`
- `yaml-cpp`
- `tinyxml2`
- `uuid`

## API

The capabilities2 server provides the following ROS2 API:

### Services

The capabilities2 server exposes the following Service API (see [capabilities2_msgs](../capabilities2_msgs/readme.md) for data types):

| Topic | Service Message | Description |
| :---  | :---            | :---        |
| `~/get_interfaces`           | `GetInterfaces.srv`         | Get the interfaces provided by the capabilities server |
| `~/get_semantic_interfaces`  | `GetSemanticInterfaces.srv` | Get the semantic interfaces provided by the capabilities server |
| `~/get_providers`            | `GetProviders.srv`          | Get the providers available on the capabilities server |
| `~/get_remappings`           | `GetRemappings.srv`         | Get the remappings for a capability |
| `~/get_capability_spec`      | `GetCapabilitySpec.srv`     | Get a raw specifications for a capability |
| `~/get_capability_specs`     | `GetCapabilitySpecs.srv`    | Get all raw specifications in the capabilities server |
| `~/get_running_capabilities` | `GetRunningCapabilities.srv`| Get the currently running capabilities |
| `~/establish_bond`           | `EstablishBond.srv`         | Establish a bond with the capabilities server to use capabilities |
| `~/use_capability`           | `UseCapability.srv`         | Use a capability |
| `~/free_capability`          | `FreeCapability.srv`        | Free a capability (when done using it, when all users are done the capability is freed) |
| `~/start_capability`         | `StartCapability.srv`       | Start a capability (this is a forceful start, and ignores use and free logic) |
| `~/stop_capability`          | `StopCapability.srv`        | Stop a capability (this is a forceful stop, and ignores use and free logic) |
| `~/register_capability`      | `RegisterCapability.srv`    | Register a capability with the capabilities server |
| `~/trigger_capability`      | `TriggerCapability.srv`    | Trigger a capability |
| `~/configure_capability`      | `ConfigureCapability.srv`    | Configure a capability with `on_start`, `on_end`, `on_success`, `on_failure` events|


### Topics

The capabilities2 server exposes the following Topics API:

| Topic | Message | Description |
| :---  | :---            | :---        |
| `~/events`  | `GetInterfaces.srv`         | Publish capability events |
| `~/bonds`   | `GetSemanticInterfaces.srv` | Maintain bonds with capability users - [Bond API](https://wiki.ros.org/bond) |

<br>

## Additional Information

- [Registering a capability](../capabilities2_server/docs/register.md)
- [Terminal based capability usage](../capabilities2_server/docs/terminal_usage.md)
