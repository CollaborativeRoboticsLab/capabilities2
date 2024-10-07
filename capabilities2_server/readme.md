# Capabilities2 Server

Capabilities2 server allows interaction with the capabilities2 API. It is a reimplementation of the capabilities package using C++ (original capabilities package was written in Python 2.7). In addition to C++ reimplementation being more resource efficient, this package extends the capabilities package features in following ways,

1. Database support for storing capabilities.
2. Capability registration through service calls.
3. Abstracted providers through a *runners* API see [capabilities2_runner](../capabilities2_runner/readme.md)

## System dependencies

The capabilities2 server depends on the following `bondcpp` ROS2 package. See the package.xml for the full list of dependencies. Notably, the capabilities2 server depends on the following system dependencies:

- `sqlite3`
- `yaml-cpp`
- `tinyxml2`
- `uuid`

## API

The capabilities2 server provides the following ROS API:

### Services

The capabilities2 server exposes the following Service API (see [capabilities2_msgs](../capabilities2_msgs/readme.md) for data types):

- `~/get_interfaces` - get the interfaces provided by the capabilities server
- `~/get_semantic_interfaces` - get the semantic interfaces provided by the capabilities server
- `~/get_providers` - get the providers available on the capabilities server
- `~/get_remappings` - get the remappings for a capability
- `~/get_capability_spec` - get a raw specifications for a capability
- `~/get_capability_specs` - get all raw specifications in the capabilities server
- `~/establish_bond` - establish a bond with the capabilities server to use capabilities
- `~/use_capability` - use a capability
- `~/free_capability` - free a capability (when done using it, when all users are done the capability is freed)
- `~/start_capability` - start a capability (this is a forceful start, and ignores use and free logic)
- `~/stop_capability` - stop a capability (this is a forceful stop, and ignores use and free logic)
- `~/register_capability` - register a capability with the capabilities server
- `~/get_running_capabilities` - get the currently running capabilities

### Topics

The capabilities2 server exposes the following Topics API:

- `~/events` -  publish capability events
- `~/bonds` -  maintain bonds with capability users - [Bond API](https://wiki.ros.org/bond)

## How to use

### Launch capabilities2 server

```bash
# can add launch args for path to packages
# for exported capabilties
# ./config/capabilities.yaml has example
ros2 launch capabilities2_server capabilities2_server.launch.py
```

### Register a capability

capabilities can be registered by exporting them in a package. The capabilities2 server will read the capabilities from the package and make them available to the user.

```xml
<!-- see std_capabilities package for examples -->
<export>
    <capability_interface>
        interfaces/cool_cap.yaml
    </capability_interface>
</export>
```

Capabilities can also be registered through a service call. This is useful for registering capabilities that are not exported in a package. The service call has been implemented as a node in the capabilities2 package. Use the node to register capabilities during runtime. THis method can also be used to update capabilities.

### Capability example

```yaml
# cool_cap.yaml
name: cool_cap
spec_version: 1
spec_type: interface
description: "This is a cool capability"
interface:
  topics:
    "/cool_topic":
      type: "std_msgs/msg/String"
      description: "This is a cool topic"
```

### User program flow

Using a capability requires the user to establish a bond with the capabilities2 server. The bond is used to maintain a persistent connection with the server.

first, inspect the available capabilities provided under this server on this robot.

```bash
ros2 service call /get_capability_specs capabilities2_msgs/srv/GetCapabilitySpecs
```

then, request a bond id to become a persistent user

```bash
ros2 service call /capabilities/establish_bond capabilities2_msgs/srv/EstablishBond

# this bond needs to be updated every second by publishing a heartbeat the bond topic
ros2 topic pub /capabilities/bonds ...
```

then request to use a capability

```bash
ros2 service call /capabilities/use_capability capabilities2_msgs/srv/UseCapability
```

This capability can be freed by calling the `free_capability` service, or just let the bond expire. The capability will be freed automatically.

## Developing

A `devcontainer` is provided for developing the capabilities2 meta-package. Dependencies need to be installed in the container. Install the dependencies with rosdep:

```bash
# in the devcontainer
rosdep install --from-paths src --ignore-src -r -y
```

### Specialising the capabilities2 server

The capabilities2 server is implemented as a [ROS2 Component](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Composition.html). The server can be specialised by composing it with another Component that adds domain logic.
