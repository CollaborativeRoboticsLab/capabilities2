# capabilities2

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://index.ros.org/doc/ros2/Releases/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Open in Visual Studio Code](https://img.shields.io/badge/vscode-dev-blue)](https://open.vscode.dev/airesearchlab/capabilities2)
![C++](https://img.shields.io/badge/Code-C++-informational?&logo=c%2b%2b)
![ROS](https://img.shields.io/badge/Framework-ROS2-informational?&logo=ROS)

A reimplementation of the [capabilities](https://github.com/osrf/capabilities) package. This package is implemented using C++17 and extends the capabilities package features. 
- [capabilities2_server](./capabilities2_server/readme.md) package contains the core of the system.
- [capabilities2_runner](./capabilities2_server/readme.md) package contains base and template classes for capability implementations.
- [capabilities2_fabric](./capabilities2_fabric/readme.md) package implements a control framework that utlizes capabilities2 system.


## System structure

![System Structure](./docs/images/system-structure.png)

## Entities

The main usage of `capabilities2` will typically involve creating or customizing capabilities through providers, interfaces and semantic interfaces. These are stored as YAML, and for more information about definitions and examples, click the links:

| Entity | Description |
| --- | --- |
| [Interfaces](./docs/interfaces.md) | The main capability specification file |
| [Providers](./docs/providers.md) | The capability provider specification file provides a mechanism to operate the capability |
| [Semantic Interfaces](./docs/semantic_interfaces.md) | The semantic interface specification file provides a mechanism to redefine a capability with semantic information |

Runners can be created using the runner API parent classes [here](./capabilities2_runner/readme.md). The capabilities service can be started using the [capabilities2_server](./capabilities2_server/readme.md) package.


## Setup Information
- [Setup Instructions with devcontainer](./docs/setup_with_dev.md)
- [Setup Instructions without devcontainer](./docs/setup.md)
- [Dependency installation for Nav2 Runners](./docs/nav2_setup.md)

## Quick Startup information

### Starting the Capabilities2 server

```bash
source install/setup.bash
ros2 launch capabilities2_server server.launch.py
```

### Starting the Capabilities2 Fabric

Start the capabilities2 server first. Then run the following on a new terminal

```bash
source install/setup.bash
ros2 launch capabilities2_fabric fabric.launch.py
```

## Additional Information
- [Motivation and Example Use Cases](./docs/motivation_and_examples.md)
- [Design Information](./docs/design.md)
- [Registering a capability](./capabilities2_server/docs/register.md)
- [Terminal based capability usage](./capabilities2_server/docs/terminal_usage.md)
- [Running test scripts](./docs/run_test_scripts.md)

## Acknowledgements

This work is based on the capabilities package developed by the Open Source Robotics Foundation. [github.com/osrf/capabilities](https://github.com/osrf/capabilities).

## Citation

If you use this work in an academic context, please cite the following publication(s):
