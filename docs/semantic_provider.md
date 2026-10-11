# (Proposed) Semantic Providers

## Description

A semantic provider is a provider specialization mechanism.

It is not the default way to represent an alternative backend or algorithm. A different backend for the same capability should usually be modeled as another ordinary provider of the same interface.

A semantic provider exists to express a provider-side delta for a specialized capability variant while preserving the standard split between:

- interface: the capability contract
- semantic interface: a refined or remapped capability contract
- provider: a concrete implementation of a capability contract
- semantic provider: a provider that extends a base provider and explicitly binds to an invokable interface

This keeps interface semantics separate from implementation choice.

## When To Use A Semantic Provider

Use a semantic provider when a specialization must change provider-owned behavior without redefining the entire provider from scratch.

Typical cases include:

- different runner implementation
- different `definition.command`
- different configuration defaults
- different dependency selection
- additional provider-level remappings
- a deployment-specific specialization of an existing provider

Do not use a semantic provider just to represent a different algorithm for the same contract.
If the contract is unchanged and only the implementation differs, create another ordinary provider for the same interface.

## Model

The recommended capability model is:

- base interface: the canonical contract
- semantic interface: a contract refinement or remapped variant of that interface
- provider: a concrete implementation of an interface contract
- semantic provider: a provider specialization that extends a base provider and must still explicitly declare which interface it implements

This enables the following valid combinations:

- `interface + provider`
- `semantic interface + provider`
- `semantic interface + semantic provider`

The following should not be inferred automatically unless explicitly declared as valid:

- every provider paired with every semantic interface redefining its base interface
- every semantic provider paired with every semantic interface in the same family

Discovery should surface only interface and provider pairings that the system can prove are invokable.

## Proposed Spec

```yaml
%YAML 1.1
---
name: turtlebot_waypoint_nav2
spec_type: semantic_provider
spec_version: 1.0
description: TurtleBot-specific specialization of the default Nav2 waypoint provider.

# The semantic provider extends an existing base provider.
extends: nav2_capabilities/WaypointRunner

# The semantic provider must explicitly bind to the interface contract it implements.
# This may be a base interface or a semantic interface.
implements: turtlebot_capabilities/TurtlebotWaypointRunner

# Optional override of the runner implementation.
runner: capabilities2_runner::WaypointRunner

# Optional provider-local remappings layered after interface-side remappings.
remappings:
	actions:
		"/navigate_to_pose": "/turtlebot4/navigate_to_pose"

# Optional dependency override.
depends_on:
	"nav2_capabilities/Nav2LaunchRunner":
		provider: "turtlebot_capabilities/TurtlebotNav2LaunchRunner"

# Optional provider definition override.
definition:
	command: "<Runner interface=turtlebot_capabilities/TurtlebotWaypointRunner provider=turtlebot_capabilities/turtlebot_waypoint_nav2 x='$x' y='$y' z='$z' qx='$qx' qy='$qy' qz='$qz' qw='$qw' />"
	configuration_parameters:
		- name: x
			type: double
			semantic_key: nav2.waypoint.x
			description: TurtleBot target x position in meters.
		- name: y
			type: double
			semantic_key: nav2.waypoint.y
			description: TurtleBot target y position in meters.
```

## Required Fields

- `name`: semantic provider name
- `spec_type`: must be `semantic_provider`
- `spec_version`: semantic provider schema version
- `extends`: the base provider being specialized
- `implements`: the interface or semantic interface this provider explicitly supports

## Optional Fields

- `description`
- `runner`
- `depends_on`
- `remappings`
- `definition`
- `relations`

## Inheritance Rules

A semantic provider should inherit from its base provider and then apply its own overrides.

Recommended precedence, lowest to highest:

1. base interface
2. semantic interface, if used
3. base provider
4. semantic provider

This means:

- interface resources are defined by the base interface
- semantic interface remappings refine the interface-side contract
- provider remappings and provider definition fields come from the base provider
- semantic provider overrides apply last

## Resolution Rules

### Runtime Resolution

The runtime should resolve an invokable capability in this order:

1. resolve the requested interface or semantic interface
2. resolve the selected provider or semantic provider
3. verify that the provider explicitly implements the requested interface, or that the pairing is explicitly declared valid
4. build the resolved interface view by applying semantic-interface remappings to the base interface
5. build the resolved provider view by inheriting the base provider and applying semantic-provider overrides
6. apply provider-side remappings to the resolved interface view
7. build the run configuration from the resolved interface plus resolved provider

### Discovery Resolution

Runnable discovery should enumerate invokable pairs, not raw providers.

For each ordinary provider:

1. emit `interface + provider` for the interface explicitly implemented by that provider
2. optionally emit `semantic interface + provider` only when that pairing is explicitly valid by declaration or compatibility policy

For each semantic provider:

1. resolve the inherited provider chain
2. emit only the interface named by the semantic provider's `implements`
3. do not infer compatibility with sibling semantic interfaces

This avoids the incorrect assumption that every semantic variant is valid for every provider variant.

## Validity Rules

The system should only expose a runnable spec when the interface and provider pairing can be proven valid.

Recommended proof rules:

### `interface + provider`

Valid when:

- the provider explicitly implements that interface

### `semantic interface + provider`

Valid when:

- the provider implements the semantic interface's base interface
- the semantic interface redefines that base interface
- the provider does not declare incompatibility
- the system policy allows that inference, or the pairing is explicitly declared

### `semantic interface + semantic provider`

Valid when:

- the semantic provider explicitly declares `implements` equal to that semantic interface
- the semantic provider's `extends` chain resolves successfully
- all inherited and overridden fields remain internally consistent

This pairing should come from explicit declaration, not inference.

## Why Explicit `implements` Matters

A semantic provider must explicitly declare `implements` even though it also declares `extends`.

That explicit binding prevents ambiguity in cases where:

- one base provider has several semantic specializations
- multiple semantic interfaces redefine the same base interface
- a semantic provider changes the execution path enough that only one specialization remains valid

Without explicit `implements`, discovery and runtime would need to guess which semantic contract the provider is meant to satisfy.

## Recommended Storage Model

If semantic providers are added to the data model, treat them as provider specializations rather than as semantic interfaces.

A reasonable conceptual schema is:

- provider
	- `name`
	- `implements`
	- `runner`
	- `depends_on`
	- `remappings`
	- `definition`
- semantic provider
	- `name`
	- `extends`
	- `implements`
	- optional overrides for `runner`, `depends_on`, `remappings`, and `definition`

This keeps contract-level inheritance separate from implementation-level inheritance.

## Recommended Constraints

- a semantic provider should extend exactly one provider
- a semantic provider should implement exactly one interface or semantic interface
- circular provider inheritance should be rejected
- missing `extends` targets should be rejected
- incompatible `implements` chains should be rejected
- dependency override semantics should be deterministic and documented

## Compatibility Notes

The standard provider for a capability remains useful and should continue to exist as the canonical, non-specialized implementation.

This gives a stable baseline:

- base interface + base provider: standard capability
- semantic interface + base provider: contract specialization without provider specialization
- semantic interface + semantic provider: contract specialization plus provider specialization

That structure supports deployment-specific tuning without collapsing interface semantics into provider choice.