# Providers

## Description

A provider is a special implementation of a capability that is specific to the current system. An interface could have multiple providers.

## Provider

```yaml
# the empty capability is an example of a capability provider
%YAML 1.1
---
name: empty
spec_type: provider
spec_version: 1.2
description: the empty capability provider for the empty interface
implements: std_capabilities/empty
runner: launch/empty.launch.py
depends_on:
  "std_capabilities/not_empty":
    provider: "std_capabilities/empty_depend"
remappings:
  topics:
    "empty": "not_empty"

# the below keys are optional

# relations are also available for providers
relations:
  # these relations are built into the provider spec version 1
  - subject: std_capabilities/empty
    predicate: implements
    object: std_capabilities/empty
  - subject: std_capabilities/empty
    predicate: depends_on
    object: std_capabilities/not_empty
  - subject: std_capabilities/empty
    predicate: remaps
    object: std_capabilities/not_empty

# providers can also have a definition using a snippet of a relevant defining language such as behavior trees
definition: 
  command: "<Runner interface=std_capabilities/empty provider=std_capabilities/empty param1='$value1' param2='$value2' />"
  configuration_parameters:
    - name: param1
      type: string
      description: static fallback value for the main text input
      semantic_key: workflow.text
    - name: param2
      type: float
      description: local configuration threshold for this provider
      semantic_key: provider.threshold
      default: 0.5
  runtime_parameters:
    input:
      - name: param1
        type: string
        description: runtime text consumed during execution
        semantic_key: workflow.text
        required: true
        satisfiable_from: [upstream, configuration]
        fallback_parameter: param1
      - name: param3
        type: string
        description: optional context value produced outside the current capability chain
        semantic_key: workflow.context.label
        required: false
        satisfiable_from: [upstream, external]
    output:
      - name: param4
        type: float
        description: score produced by this provider for downstream consumption
        semantic_key: workflow.score
        aliases: [score]
```

## Parameter Role Fields

The provider definition separates `configuration_parameters` from `runtime_parameters`.
That split is the primary contract.

The missing piece is how to describe a runtime input that may also be satisfied by static configuration or a default value.

The spec keeps the existing sections and adds optional fields to each parameter entry.

### Optional Fields

- `semantic_key`: Stable semantic identifier used for cross-capability matching and graph relations. If omitted, `name` is used as the fallback identity.
- `required`: Declares whether the parameter must be satisfied for valid execution. This is primarily meaningful for runtime inputs.
- `satisfiable_from`: Declares allowed sources for satisfying a runtime input. Recommended values are any combination of the following:
  - `upstream`: The runtime input may be satisfied by a matching runtime output from an upstream capability.
  - `configuration`: The runtime input may be satisfied by a matching configuration parameter from the same provider.
  - `external`: The runtime input may be satisfied by an external source outside the current capability chain.
  - `default`: The runtime input may be satisfied by a default value declared on the parameter itself or on a fallback configuration parameter.
- `fallback_parameter`: Names the configuration parameter that may satisfy this runtime input when no upstream producer is connected.
- `aliases`: Optional alternative names that may be used during semantic matching or migration.
- `default`: Optional default value when the parameter can be satisfied locally without an upstream producer.

### Semantics

- Parameter role is still determined by where the parameter is declared: `configuration_parameters`, `runtime_parameters.input`, or `runtime_parameters.output`.
- Configuration parameters do not create cross-capability data-flow edges by themselves.
- Runtime output to runtime input matches create candidate data-flow edges.
- A runtime input may share the same `name` as a configuration parameter if the runtime input explicitly declares how configuration may satisfy it.
- `semantic_key` is the preferred identity for retrieval and validation. `name` remains the local binding name used by the runner.

### Example: Common Parameter With Configuration Fallback

```yaml
definition:
  command: "<Runner interface=prompt_capabilities/PromptTextRunner provider=prompt_capabilities/PromptTextRunner text='$text' uuid='$uuid' flush='false' />"
  configuration_parameters:
    - name: text
      type: string
      description: Static fallback text used when no upstream runtime text is provided.
      semantic_key: prompt.text
    - name: uuid
      type: string
      description: Optional cache identifier used to bind prompt state across capabilities.
      semantic_key: prompt.cache.uuid
    - name: flush
      type: boolean
      description: Flush the prompt cache after execution.
      semantic_key: prompt.cache.flush
      default: false
  runtime_parameters:
    input:
      - name: text
        type: string
        description: Text payload consumed during execution.
        semantic_key: prompt.text
        required: true
        satisfiable_from: [upstream, configuration]
        fallback_parameter: text
    output:
      - name: response
        type: string
        description: Prompt response returned by the provider.
        semantic_key: prompt.response
```

### Interpretation For Common Parameters

When the same logical datum appears in both config and runtime sections, treat it as one semantic parameter with multiple allowed satisfaction paths:

- runtime flow is preferred when an upstream capability produces a matching output,
- configuration fallback is allowed only when the runtime input explicitly permits it,
- default satisfaction is allowed only when declared on the receiving parameter or its fallback configuration parameter.

This keeps execution binding simple while still allowing retrieval and validation to reason about real data flow.
