# Capabilities2 Server

This is the capabilities2 server. It allows interaction with the capabilities2 API. It is a reimplementation of the capabilities package using CPP. The CPP implementation is more efficient. Secondly, this package extends the capabilities package features.

## System dependencies

The capabilities2 server depends on the following system dependencies:

- sqlite3

## API

The capabilities2 server provides the following API:

### Services

The capabilities2 server exposes the following Service API:

- `~/get_interfaces`
- `~/get_semantic_interfaces`
- `~/get_providers`
- `~/get_remappings`
- `~/get_capability_specs`

### Topics

The capabilities2 server exposes the following Topics API:

- `~/events` -  publish capability events
- `~/bonds` -  maintain bonds with capability users

## How to use

first, inspect the available capabilities

```bash
ros2 service call /get_capability_specs capabilities2_msgs/srv/GetCapabilitySpecs
```

then, request a bond id to become a persistent user

```bash
ros2 service call /capabilities/establish_bond capabilities2_msgs/srv/EstablishBond

# this bond needs to be updated every second by publishing a heartbeat the the bond topic
ros2 topic pub /capabilities/bonds ...
```

then request to use a capability

```bash
ros2 service call /capabilities/use_capability capabilities2_msgs/srv/UseCapability
```

This capability can be freed by calling the `free_capability` service.
