# Creating Runner Plugins

The runner API is designed to be extensible, so that specific runners can be created on a per provider basis. Another feature that can be implemented is cross-runner communication, which could allow capabilities to be combined at runtime to perform more complex tasks. This could overcome limitations in robot programming in which there is a common need for extensive pre-definition of tasks.

## Inheritance

A runner is a plugin that inherits from the `RunnerBase` class. The `RunnerBase` class is a pure virtual class that defines the interface for a runner. The `RunnerBase` class has the following methods:

1. `start` - starts the runner
1. `stop` - stops the runner
1. `trigger` - triggers the runner

Multiple classes are provided to inherit from, depending on the type of runner you want to create. A notable example is the `ActionRunner` class, which is used to create runners that run actions.

### Export with Pluginlib

The runner should be exported as a plugin using the `PLUGINLIB_EXPORT_CLASS` macro.

```cpp
#include <capabilities2_runner/runner_base.hpp>
#include <pluginlib/class_list_macros

...

PLUGINLIB_EXPORT_CLASS(capabilities2_runner::MyRunner, capabilities2_runner::RunnerBase)
```
