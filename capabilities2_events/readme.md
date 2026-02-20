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
| type | the type of connection (e.g., ON_START, ON_STOP, ON_SUCCESS, ON_FAILURE) |
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
3. When a capability emits an event (e.g., STARTED), the event system checks for any connections matching that event type from the source capability
4. For each matching connection, the target capability is invoked accordingly (e.g., started, stopped, etc.)

### Event Emission Triggering

There are two ways events can be tracked for emission:

1. the connection is for STARTED or STOPPED events - these are persistently tracked and become dependencies between capabilities
2. the connection is for SUCCEEDED or FAILED events - these are one-shot events that are emitted when a running capability is triggered to succeed or fail

This works out to mean that a STARTED capability will implicitly start its dependent capabilities, and immediately run their trigger action.

### Event trigger ID

The event trigger ID is a unique identifier for the event connection. It also needs to account for multiple clients. It is constructed using URI format from the bond ID and the trigger ID specified by the user:

```
string connection_id = '<bond_id>/<trigger_id>'
uuid bond_id -> provided on bond establishment
string trigger_id -> user specified id for the connection
event_id = connection_id = bond_id + '/' + trigger_id
```

### Notes

- a single publisher is used for all event messages, this is at the top level (capabilities2_server)
- events are fired from runners

#### To Do

- [x] instantiate event publisher in server since publisher is defined there
- [x] store the event class in the api, since api owns runners
- [x] then pass event system api object pointer to runners to allow firing events from runners, can be passed to event node parent class
- [x] fix duplicate - event, trigger, runner - id - just use names?
- [x] remove event_opts type usage

- [x] implement event node class\
- [x] inherit event node in runner base class
- [x] generalise event connections to use a map of event types to connection objects
- [x] add ability to disconnect events
- [x] add ability to list current event connections
- [x] add ability to connect multiple events of same type to different targets

- [ ] add ability to specify parameters for target capability on event connection

- [ ] add ability to specify event connections in capability definition files
