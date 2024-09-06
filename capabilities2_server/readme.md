# Capabilities2 Server

This is the capabilities2 server. It allows interaction with the capabilities2 API. It is a reimplementation of the capabilities package using CPP. The CPP implementation is more efficient. Secondly, this package extends the capabilities package features.

## System dependencies

The capabilities2 server depends on the following system dependencies:

- `sqlite3`

## API

The capabilities2 server provides the following API:

### Services

The capabilities2 server exposes the following Service API:

- `~/get_interfaces`
- `~/get_semantic_interfaces`
- `~/get_providers`
- `~/get_remappings`
- `~/get_capability_specs`
- `~/establish_bond`
- `~/use_capability`
- `~/free_capability`

### Topics

The capabilities2 server exposes the following Topics API:

- `~/events` -  publish capability events
- `~/bonds` -  maintain bonds with capability users - [Bond API](https://wiki.ros.org/bond)

## How to use

### launch capabilities2 server

```bash
# can add launch args for path to packages
# for exported capabilties
# ./config/capabilities.yaml has example
ros2 launch capabilities2 capabilities2_server.launch.py
```

### register a capability

capabilities can be registered by exporting them in a package. The capabilities2 server will read the capabilities from the package and make them available to the user.

```xml
<!-- see std_capabilities package for examples -->
<export>
    <capability_interface>
        interfaces/cool_cap.yaml
    </capability_interface>
</export>
```

Capabilities can also be registered through a service call. This is useful for registering capabilities that are not exported in a package. The service call has been implemented as a node in the capabilities2 package. Use the node to register capabilities during runtime.

### user program flow

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
