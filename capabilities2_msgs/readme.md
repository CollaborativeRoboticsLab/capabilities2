# capabilities2_msgs

Message service and action types for the capabilities2 interface.

## Message Types

- `Capability.msg` - A message type for a capability of a robot.
- `CapabilityCommand.msg` - A message type for a command to a robot.
- `CapabilityResponse.msg` - A message type for a response from a robot related to a capability.
- `NaturalCapability.msg` - A message type for a natural capability of a robot.
- `Remapping.msg` - A message type for a key/value remapping of a capability resource.
- `CapabilitySpec.msg` - A message type for the specification of a capability.
- `CapabilityEvent.msg` - A message type for an event related to a capability.
- `RunnableCapability.msg` - A message type for a runnin a capability.

## Service Types

- `EstablishBond.srv` - A service type for establishing a bond between a robot and a client.
- `FreeCapability.srv` - A service type for freeing a capability of a robot.
- `GetCapabilitySpec.srv` - A service type for getting the specification of a capability of a robot.
- `GetCapabilitySpecs.srv` - A service type for getting the specifications of all capabilities of a robot.
- `GetInterfaces.srv` - A service type for getting the interfaces of a robot.
- `GetProviders.srv` - A service type for getting the providers of a capability of a robot.
- `GetRemappings.srv` - A service type for getting the remappings of a capability of a robot.

## Action Types

- `Capability.action` - An action type for a capability request of a robot.
