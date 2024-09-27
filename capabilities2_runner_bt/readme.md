# capabilities2_runner_bt

Behavior tree runners for [capabilities2](../readme.md). This package provides a runner for the capabilities2 package that uses behavior trees to combine and execute composite runners in the capabilities framework.

## Runners

- `BTRunnerBase` - A base class for behavior tree runners. This class is used to implement the behavior tree runner for the capabilities2 package. It inherits from the [`ActionRunner`](../capabilities2_runner/readme.md) to join ROS action and behavior tree action concepts.
- `BTFactoryRunner` - Load a behavior tree and run it as a capabilities2 runner (this just means that it runs in the capabilities2_server node).
