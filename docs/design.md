## Design

The new capabilities package is designed to be more efficient and extensible. The functions are implemented as plugins, which can be loaded at runtime. The execution of providers is abstracted using an API called runners. The runners can manage more arbitrary provider operation which can include performing actions or services, or even running other capabilities. The capability models are stored in a database, This will allow various feature improvements such as hot reloading, state persistence, and model extension. Another possible feature is to create more complex ontological relationships between capabilities such as prerequisites, conflicts, or RDF triples (for example, `grasp` results in `holding`, or `pick` is a type of `manipulation`, or `grasp` is incompatible with `push`).

### Bonds

The bonds feature as implemented in `capabilities` and reimplemented in `capabilities2` allows applications to overlap functions by requesting use of the same resources. When all references to a resource are released, the resource is stopped. This has the added benefit of creating an idle state for the robot in which the robot computing resources are not being used. As a result of this, the robot could be more energy-efficient, and the robot could be more responsive to new tasks.

### Runners

The runners feature which is new in `capabilities2` allows capabilities to be executed in a more diverse manner. This could include running actions, services, or even other capabilities. This allows capabilities to be used like actions and not just started and stopped by the capabilities service. The Runner API is designed to be extensible, so that specific runners can be created on a per provider basis. Another feature that can be implemented is cross-runner communication, which could allow capabilities to be combined at runtime to perform more complex tasks. This could overcome limitations in robot programming in which there is a common need for extensive pre-definition of tasks.

#### launch runner

To enable the launch functionality from `capabilities` in `capabilities2`, a `launch runner` has been implemented. Due to the design of the launch system in ROS2, it was necessary to create a `launch_proxy` node which uses the `python` launch API to start and stop launch files. The runner allows uses an action to start and stop launch files, and keep track of running launch files.
