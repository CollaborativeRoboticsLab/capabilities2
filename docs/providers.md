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
spec_version: 1.1
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
definition: |
  <root>
    <sequence>
      <action name="empty"/>
    </sequence>
  </root>
```
