# capabilities2

[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue)](https://index.ros.org/doc/ros2/Releases/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Open in Visual Studio Code](https://img.shields.io/badge/vscode-dev-blue)](https://open.vscode.dev/airesearchlab/capabilities2)

A reimplementation of the [capabilities](https://github.com/osrf/capabilities) package. This package is implemented using CPP and extends the capabilities package features.

## System structure

![System Structure](./docs/images/system-structure.png)

## Design

The new capabilities package is designed to be more efficient and extensible. The functions are implemented as plugins, which can be loaded at runtime. The execution of providers is abstracted using an API called runners. The runners can manage more arbitrary provider operation which can include performing actions or services, or even running other capabilities. The capability models are stored in a database, This will allow various feature improvements such as hot reloading, state persistence, and model extension. Another possible feature is to create more complex ontological relationships between capabilities such as prerequisites, conflicts, or RDF triples (for example, `grasp` results in `holding`, or `pick` is a type of `manipulation`, or `grasp` is incompatible with `push`).

### Bonds

The bonds feature as implemented in `cababilities` allows applications to overlap functions by requesting use of the same resources. When all references to a resource are released, the resource is stopped. This has the added benefit of creating an idle state for the robot in which the robot computing resources are not being used. As a result of this, the robot could be more energy-efficient, and the robot could be more responsive to new tasks.

### Runners

The runners feature is new in `cababilities2`. It allows capabilities to be executed in a more arbitrary way. This could include running actions, services, or even other capabilities. This allows capabilities to be used like actions and not just started and stopped by the capabilities service. The Runner API is designed to be extensible, so that specific runners can be created on a per provider basis. Another feature that can be implemented is cross-runner communication, which could allow capabilities to be combined at runtime to perform more complex tasks. This could overcome limitations in robot programming in which there is a common need for extensive pre-definition of tasks.

#### launch runner

To enable the launch functionality from `capabilities` in `capabilities2`, a `launch runner` has been implemented. Due to the design of the launch system in ROS2, it was necessary to create a `launch_proxy` node which uses the `python` launch API to start and stop launch files. The runner allows uses an action to start and stop launch files, and keep track of running launch files.

## Motivation

The capabilities package was originally implemented using Python. This package is a reimplementation of the capabilities package using CPP. The CPP implementation is more efficient. Secondly, this package extends the capabilities package features.

The main reasons for this are:

1. To allow the capabilities service to provide details of the robot to other robots, a UI or App, or a supervisory AI with a conversational style.
1. To allow packages to register their capabilities with the capabilities service using a service API or a spawner.
1. To allow capabilities to be used like actions and not just started and stopped by the capabilities service.
1. To add state to the capabilities service using a database. This allows the capabilities service or the robot to be restarted without losing state, and hot reloading.
1. To allow the capabilities service to be used as a library.

### Example use cases

- A generic GenAI could derive a plan from the capabilities of a robot, to perform more general multi-step tasks. This could be achieved by using the capabilities as a knowledge base. This may solve problems with GenAI integration, where the GenAI has to be trained on a specific robot, or there is a significant delay in the GenAI communicating with the robot - since the GenAI does not actually execute the task, it is used to suggest a plan and supervise the robot.
- A robot encounters another robot of unknown origin and asks it what it can do. The capabilities could be communicated using M2M communication or speech. This could allow robots from different manufacturers to work together.
- A robot is asked to do a task it has never done before. This could be achieved by combining capabilities in new ways. or a robot is asked to do a task it has done before but with a different context. This could be achieved by changing the parameters of capabilities.
- A universal remote control for robots could be created using the capabilities as a standardised interface. This might be useful for robot subsystem integrators. This could also be used to create a robot app store, or standardised sensor and actuator interfaces.

## acknoledgements

This work is based on the capabilities package developed by the Open Source Robotics Foundation.

## Citation

If you use this work in an academic context, please cite the following publication(s):
