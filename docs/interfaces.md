# Interfaces

## Description

An capability interface describes the capability. It relates ROS primitives to the capability.

## Interface

```yaml
# the empty capability is an example of a capability interface
%YAML 1.1
---
name: empty
spec_type: interface
spec_version: 1.1
description: the empty capability interface helps to test the capability server
interface:
  parameters:
    "empty":
      type: None
      description: empty
  topics:
    "empty":
      type: std_msgs/msg/Empty
      description: empty
  services:
    "empty":
      type: std_srvs/srv/Empty
      description: empty
  actions:
    "empty":
      type: std_srvs/action/Empty
      description: empty

# the below keys are optional

# relations are a list of relations to other capability entities
relations:
    # an example of a relation
  - subject: some_package/some_capability
    predicate: blocks
    object: some_other_package/some_other_capability
```
