# capabilities2

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://index.ros.org/doc/ros2/Releases/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Open in Visual Studio Code](https://img.shields.io/badge/vscode-dev-blue)](https://open.vscode.dev/airesearchlab/capabilities2)
![C++](https://img.shields.io/badge/Code-C++-informational?&logo=c%2b%2b)
![ROS](https://img.shields.io/badge/Framework-ROS-informational?&logo=ROS)

A reimplementation of the [capabilities](https://github.com/osrf/capabilities) package. This package is implemented using C++17 and extends the capabilities package features. See the [capabilities2_server](./capabilities2_server/readme.md) package for the main system component.

[More Information about Motivation and Example Use Cases](./docs/motivation_and_examples.md)

## System structure

![System Structure](./docs/images/system-structure.png)

## Entities

The main usage of `capabilities2` will typically involve creating capabilities through providers, interfaces and semantic interfaces. Which are represented in specification file stored as YAML, see the definitions and examples for each:

| Entity | Description |
| --- | --- |
| [Interfaces](./docs/interfaces.md) | The main capability specification file |
| [Providers](./docs/providers.md) | The capability provider specification file provides a mechanism to operate the capability |
| [Semantic Interfaces](./docs/semantic_interfaces.md) | The semantic interface specification file provides a mechanism to redefine a capability with semantic information |

See [docs](./docs/) for the format of these entities or click the links above. Runners can be created using the runner API parent classes [here](./capabilities2_runner/readme.md). The capabilities service can be started using the [capabilities2_server](./capabilities2_server/readme.md) package.

## Design

The new capabilities package is designed to be more efficient and extensible. The functions are implemented as plugins, which can be loaded at runtime. The execution of providers is abstracted using an API called runners. The runners can manage more arbitrary provider operation which can include performing actions or services, or even running other capabilities. The capability models are stored in a database, This will allow various feature improvements such as hot reloading, state persistence, and model extension. Another possible feature is to create more complex ontological relationships between capabilities such as prerequisites, conflicts, or RDF triples (for example, `grasp` results in `holding`, or `pick` is a type of `manipulation`, or `grasp` is incompatible with `push`).

### Bonds

The bonds feature as implemented in `capabilities` and reimplemented in `capabilities2` allows applications to overlap functions by requesting use of the same resources. When all references to a resource are released, the resource is stopped. This has the added benefit of creating an idle state for the robot in which the robot computing resources are not being used. As a result of this, the robot could be more energy-efficient, and the robot could be more responsive to new tasks.

### Runners

The runners feature which is new in `capabilities2` allows capabilities to be executed in a more diverse manner. This could include running actions, services, or even other capabilities. This allows capabilities to be used like actions and not just started and stopped by the capabilities service. The Runner API is designed to be extensible, so that specific runners can be created on a per provider basis. Another feature that can be implemented is cross-runner communication, which could allow capabilities to be combined at runtime to perform more complex tasks. This could overcome limitations in robot programming in which there is a common need for extensive pre-definition of tasks.

#### launch runner

To enable the launch functionality from `capabilities` in `capabilities2`, a `launch runner` has been implemented. Due to the design of the launch system in ROS2, it was necessary to create a `launch_proxy` node which uses the `python` launch API to start and stop launch files. The runner allows uses an action to start and stop launch files, and keep track of running launch files.


## Using Capabilities package (Development and Deployment)

The current package has been tested on Ubuntu and ROS2 Rolling and Jazzy.

### Basic setup

Create the workspace folder by running following commands in the terminal.

```bash
mkdir -p /home/$USER/capabilities_ws/src
cd /home/$USER/capabilities_ws/src
```

Unzip and copy the `capabilities2` folder into the `/home/$USER/capabilities_ws/src` folder. (Final path should be like `/home/$USER/capabilities_ws/src/capabilities2`)

### When using devcontainer

A `devcontainer` is provided for developing the capabilities2 meta-package. The container can be used with [Microsoft Visual Studio Code](https://code.visualstudio.com/) and [Remote Development Extention](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack). Dependencies need to be installed in the container. Install the dependencies with rosdep:

```bash
# in the devcontainer
rosdep install --from-paths src --ignore-src -r -y
```

### When using without the devcontainer

On the terminal run the following command to identify the $USER and note down the value

```bash
echo $USER
```

Then open the `/home/$USER/ROS/capabilities_ws/src/capabilities2/capabilities2_server/config/capabilities.yaml` with an available text editor. Either gedit or nano can be used.

```sh
nano /home/$USER/ROS/capabilities_ws/src/capabilities2/capabilities2_server/config/capabilities.yaml
```

In the opened file, replace $USER value in lines 9 & 10 with above identified value for $USER. if the $USER is ubuntu, those lines should be

```yaml
      - /home/ubuntu/capabilities_ws/src
      - /home/ubuntu/capabilities_ws/src/capabilities2
```

Save the file and close.

Move the terminal to workspace root and install dependencies.

```bash
cd /home/$USER/capabilities_ws
```
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Building

Use Colcon to build the package:

```bash
colcon build
```

### Starting the Capabilities2 server

```bash
source install/setup.bash
ros2 launch capabilities2_server capabilities2_server.launch.py
```

### Terminal based bond creation

Read more about this [here](../capabilities2/capabilities2_server/readme.md) 

### Running Test cases

Read more about this [here](../capabilities2/capabilities2_server/readme.md) 

## Acknowledgements

This work is based on the capabilities package developed by the Open Source Robotics Foundation. [github.com/osrf/capabilities](https://github.com/osrf/capabilities).

## Citation

If you use this work in an academic context, please cite the following publication(s):
