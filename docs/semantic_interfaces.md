# Semantic Interfaces

## Description

A semantic interface is a special type of capability interface that is used to define the semantics of an interface

## Semantic Interface

```yaml
# the empty capability is an example of a capability interface
# redefines the empty interface
%YAML 1.1
---
name: not_empty
spec_type: semantic_interface
spec_version: 1
description: the not empty semantic empty interface
redefines: std_capabilities/empty
global_namespace: not
remappings:
  parameters:
    "empty": "not_empty"
  topics:
    "empty": "not_empty"
  services:
    "empty": "not_empty"
  actions:
    "empty": "not_empty"

# the below keys are optional

# relations are also available for semantic interfaces
relations:
  # these relations are built into the semantic interface spec version 1
  - subject: std_capabilities/empty
    predicate: redefines
    object: std_capabilities/not_empty
  - subject: std_capabilities/empty
    predicate: remaps
    object: std_capabilities/not_empty

```
