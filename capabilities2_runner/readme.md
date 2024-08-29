# capabilities2_runner

Provide base class runners for common capabilities like nav2, moveit2, etc. Plugins extend the execution functionality of the `capabilities` system. The ROS1 implementation used launch files to start capabilities. The ROS2 implementation uses runners to start capabilities. This allows for more flexibility in how capabilities are started and stopped, or how they are managed.

## Runners

- `base runner` - base class for runners implementing the `Runner` interface which comprises of `start`, `stop`.
- `action runner` - runner for capabilities that are implemented as actions. This is also a base class as most runners will be actions.
- `launch runner` - runner for capabilities that are implemented as launch files.
