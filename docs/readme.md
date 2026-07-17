# Docs

For detailed information about the code, build the code documentation using Doxygen. see [`capabilities2_documentation`](../capabilities2_documentation/).

```bash
# in the devcontainer
cd capabilities2_documentation
doxygen Doxyfile
```

## Basic Tutorial

A basic tutorial can be found in [here](./basic_tutorial.md). This tutorial will guide you through the process of creating a simple capability and running it.

## Capabilities Server

The [capabilities server](../capabilities2_server/readme.md) is the main node in the capabilities2 package. It is responsible for managing capabilities, providers, and semantic interfaces. It is also responsible for starting and stopping capabilities.

## Capabilities Runner

A capabilities runner is a plugin that allows capabilities to be executed in a more arbitrary way. Base runner classes can be used to create custom runners for a given application. See the [capabilities2_runner](../capabilities2_runner/readme.md) package for more information.

## Capabilities Messages

This package contains messages for the capabilities2 package. See the [capabilities2_msgs](../capabilities2_msgs/readme.md) package for more information.

## Launch Runner Status

The `LaunchRunner` interface is still kept in the `capabilities2_runner` package, but launch-file execution support is intentionally deferred because of incompatibilities with the current ROS2 launch system. Documentation for launch-based capability execution will be restored once the replacement solution is ready.
