# capabilities2_runner plugin API

provides `runner` API for abstract provision of capabilities. Plugins extend the execution functionality of the `capabilities` system. The ROS1 implementation used launch files to start capabilities. The ROS2 implementation uses runners to start capabilities. This allows for more flexibility in how capabilities are started and stopped, or how they are managed, and operate.

## Runners

- `base runner` - base class for runners implementing the `Runner` interface which comprises of `start`, `stop`.
- `action runner` - runner for capabilities that are implemented as actions. This is also a base class for as most runners will be actions.
- `launch runner` - runner for capabilities that are implemented as launch files.
- `multi action runner` - runner for capabilities that are implemented as multiple actions.

### launch runner

The `launch runner` inherits from the `action runner` and is a special case. To instatiate this runner, provide a launch file path as the `runner` tag in the capability provider.

```yaml
# provider ...
name: my_provider
spec_version: 1
spec_type: provider
implements: my_capability
# the runner to use is an exported plugin name based on RunnerBase
runner: path/to/launch_file.launch.py
```

## Inheritance Diagram

An example of the inheritance diagram for the runners is shown below. The `RunnerBase` class is the base class for all runners. The `ActionRunner` class is the base class for runners that are implemented as actions. The example shows a waypoint capability that is implemented as a single action. The `WaypointRunner` class inherits from the `ActionRunner` class.

![inheritance diagram](./docs/images/inheritance-diagram.png)

## creating a new runner

New runners can be created to perform new capabilities. The runner can be specified in a capability provider as the `runner` tag:

```yaml
# provider ...
name: my_provider
spec_version: 1
spec_type: provider
implements: my_capability
# the runner to use is an exported plugin name based on RunnerBase
runner: capabilities2_runner::MyRunner
```

The runner must inherit from the RunnerBase or the ActionRunner class. The runner must implement the `start` and `stop` methods, and then be registered as a plugin.
