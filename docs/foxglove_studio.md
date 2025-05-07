# Dependency Installation with Foxglove Studio

We utilize foxglove studio to visalize information about the Capabilities2 system.

Event system has been defined in capabilities2_events package and foxglove-bridge has been set as a dependency of capabilities2_events for ease of installation. We have selected foxglove-bridge over rosbridge due to performance improvements foxglove-bridge has over rosbridge due to former being written in C++ and latter being in Python.

To visualize data, download foxglove-studio from [website](https://foxglove.dev/download) and create a free account when signing in.