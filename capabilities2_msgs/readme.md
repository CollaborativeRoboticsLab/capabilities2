# capabilities2_msgs

Message, service, and action types for the capabilities2 interface.

## Message Types

- `Capability.msg` - A message type for a capability of a robot.
- `CapabilityCommand.msg` - A message type for a command to a robot.
- `CapabilityEvent.msg` - A message type for an event related to a capability.
- `CapabilityResponse.msg` - A message type for a response from a robot related to a capability.
- `CapabilitySpec.msg` - A message type for the specification of a capability.
- `NaturalCapability.msg` - A message type for a natural capability of a robot.
- `Remapping.msg` - A message type for a key/value remapping of a capability resource.
- `RunningCapability.msg` - A message type for a running capability.

## Service Types

- `EstablishBond.srv` - A service type for establishing a bond between a robot and a client.
- `FreeCapability.srv` - A service type for freeing a capability of a robot.
- `GetCapabilitySpec.srv` - A service type for getting the specification of a capability of a robot.
- `GetCapabilitySpecs.srv` - A service type for getting the specifications of all capabilities of a robot.
- `GetInterfaces.srv` - A service type for getting the interfaces of a robot.
- `GetProviders.srv` - A service type for getting the providers of a capability of a robot.
- `GetRemappings.srv` - A service type for getting the remappings of a capability of a robot.
- `GetRunningCapabilities.srv` - A service type for getting the running capabilities of a robot.
- `GetSemanticInterfaces.srv` - A service type for getting the semantic interfaces.
- `RegisterCapability.srv` - A service type for registering a capability with the server.
- `StartCapability.srv` - A service type for starting a capability.
- `StopCapability.srv` - A service type for stopping a capability.
- `UseCapability.srv` - A service type for using a capability.

## Action Types

- `Capability.action` - An action type for an encapsulated capability action.
- `Launch.action` - An action type for launching a launch file.
