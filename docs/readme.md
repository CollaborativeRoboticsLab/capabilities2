# Docs

For detailed information about the code, build the code documentation using Doxygen. see [`capabilities2_documentation`](./capabilities2_documentation/).

```bash
# in the devcontainer
cd capabilities2_documentation
doxygen Doxyfile
```

## Capabilities Server

The capabilities server is the main node in the capabilities2 package. It is responsible for managing capabilities, providers, and semantic interfaces. It is also responsible for starting and stopping capabilities.

## Capabilities Runner

A capabilities runner is a plugin that allows capabilities to be executed in a more arbitrary way. Base runner classes can be used to create custom runners for a given application.

## Capabilities Messages

This package contains messages for the capabilities2 package.

## Capabilities Launch Proxy

The capabilities launch proxy is a node that uses the Python launch API to start and stop launch files. It is used by the launch runner to start and stop launch files.
