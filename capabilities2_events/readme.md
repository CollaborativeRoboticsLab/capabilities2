# Capabilities2 Events

This package provides an event handling subsystem for the Capabilities2 framework. Events form the basis of inter-capability communication, allowing capabilities to respond to state changes in other capabilities. This facilitates the creation of complex behaviours through the composition of simpler capabilities.

## features

- Uses capability event messages
- helpers for integration with the capabilities2_runner, and capabilities2_server packages
- system for connecting capabilities together based on events. e.g. capability state transitions occur based on events from other capabilities

## Model

The event system is based around the concepts of nodes, connections, events.

### Node

Represents a capability that can emit events and has connections to other capabilities.

| model data | description |
|---|---|
| id | unique identifier for the node |
| connections | a map of connections. The source i always *this* node |

### Connection

Represents the link between two nodes.

| model data | description |
|---|---|
| type | the type of connection (e.g., STARTED, STOPPED, SUCCEEDED, FAILED) |
| source | the source capability that emits the event |
| target | the target capability to invoke when the event is emitted |

### Event

Represents a specific event that can be emitted by a node (e.g., STARTED, STOPPED, SUCCEEDED, FAILED)

### event types

State transitions are as defined in the event code message:

```
IDLE, // initial state
STARTED, // when capability is started
STOPPED, // when capability is stopped
// TRIGGERED, // when capability is TRIGGERED
FAILED, // when capability has FAILED
SUCCEEDED // when capability has SUCCEEDED
etc..
```

## Flow

1. A user establishes a bond with the capabilities2_server
2. The user connects capabilities together by specifying event connections (source capability, event type, target capability)
3. The server namespaces the connection using the bond id, the source capability instance id, and the target capability instance id
4. When a capability emits an event (e.g., STARTED), the event system checks for any connections matching that event type from the source capability, bond, and source instance id
5. For each matching connection, the target capability is invoked accordingly (e.g., started, stopped, etc.)

## Example usage

After a bond is established and both capabilities are running, a client can connect them through `ConnectCapability.srv`.

```bash
ros2 service call /capabilities/connect_capability capabilities2_msgs/srv/ConnectCapability "{
	bond_id: '<bond-id>',
	connection: {
		type: {code: 1},
		source: {
			capability: 'demo_pkg/source_capability',
			provider: 'demo_pkg/source_provider',
			instance_id: 'source_instance'
		},
		target: {
			capability: 'demo_pkg/target_capability',
			provider: 'demo_pkg/target_provider',
			instance_id: 'target_instance',
			parameters: [
				{
					key: 'mode',
					value: ['auto'],
					type: 3
				}
			]
		}
	}
}"
```

in this example, `type.code: 1` corresponds to `STARTED`. when the source capability emits a `STARTED` event for `source_instance`, the target capability is triggered for `target_instance`. any parameters specified on the target capability are carried through the connection and merged with event parameters at emission time.

### Event namespacing

event connections are scoped in three ways:

1. by bond id, so one client cannot trigger another client's event wiring
2. by source capability instance id, so only the matching running source instance emits to that connection
3. by target capability instance id, so the event carries the intended destination instance when the callback is invoked

the connection identifier built by the server currently has this format:

```text
bond_id/source_instance_id/target_instance_id
```

the source capability runner stores this connection id and, during event emission, matches on:

- event type
- bond id
- source instance id

when a match is found, the target capability and target instance id are forwarded through the callback. the published event message also uses the source instance id as the `trigger_id` field.

### Event Emission Triggering

There are two ways events can be tracked for emission:

1. the connection is for STARTED or STOPPED events - these are persistently tracked and become dependencies between capabilities
2. the connection is for SUCCEEDED or FAILED events - these are one-shot events that are emitted when a running capability is triggered to succeed or fail

This works out to mean that a STARTED capability will implicitly start its dependent capabilities, and immediately run their trigger action.

### Event trigger ID

the published event message exposes a `trigger_id` field. in the current implementation this is populated from the source capability instance id, not from a separate user-defined trigger namespace.

```text
connection_id = bond_id + '/' + source_instance_id + '/' + target_instance_id
published trigger_id = source_instance_id
```

### Notes

- a single publisher is used for all event messages, this is at the top level (capabilities2_server)
- events are fired from runners

#### To Do

- [x] add ability to specify parameters for target capability on event connection
- [ ] add ability to specify event connections in capability definition files
