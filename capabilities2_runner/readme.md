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

The runner should inherit from the `RunnerBase` or another like the `ActionRunner` class. The runner must implement the `start`, `stop`, and `trigger` methods, and then be registered as a plugin, using the `PLUGINLIB_EXPORT_CLASS` macro.

### Base Runner

The `RunnerBase` class is the base class for all runners. It provides the `start`, `stop`, and `trigger` methods. The `start` method is used to start the runner, the `stop` method is used to stop the runner, and the `trigger` method is used to trigger the runner. See the `RunnerBase` class definition function templates below:

```cpp
namespace capabilities2_runner
{

  class RunnerBase
  {
  public:
    // start the runner
    virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                     std::function<void(const std::string&)> on_started = nullptr,
                     std::function<void(const std::string&)> on_terminated = nullptr,
                     std::function<void(const std::string&)> on_stopped = nullptr) = 0;

    // stop the runner
    virtual void stop() = 0;

    // trigger the runner
    virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
    trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) = 0;
  };

} // namespace capabilities2_runner
```

## Experimental Runners

The `capabilities2_runner` package provides experimental runners that can be used to start capabilities. These runners are not fully tested and may not work as expected. The experimental runners are:

- `capabilities2_runner::EnCapRunner` - runner that provides a capability action interface that encapsulates another action.
- `capabilities2_runner::MultiActionRunner` - runs multiple actions in parallel.
