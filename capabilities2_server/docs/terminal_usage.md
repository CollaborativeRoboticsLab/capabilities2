## Terminal based capability usage

Using a capability requires the user to establish a bond with the capabilities2 server. The bond is used to maintain a persistent connection with the server.

first, inspect the available capabilities provided under this server on this robot.

```bash
ros2 service call /capabilities/get_capability_specs capabilities2_msgs/srv/GetCapabilitySpecs
```

then, request a bond id to become a persistent user

```bash
ros2 service call /capabilities/establish_bond capabilities2_msgs/srv/EstablishBond

# this bond needs to be updated every second by publishing a heartbeat the bond topic. Replace the ... with value received from above call
ros2 topic pub /capabilities/bonds ...
```

then request to use a capability

```bash
ros2 service call /capabilities/use_capability capabilities2_msgs/srv/UseCapability
```

This capability can be freed by calling the `free_capability` service, or just let the bond expire. The capability will be freed automatically.
