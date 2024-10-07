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
#include <pluginlib/class_list_macros.hpp>

...

PLUGINLIB_EXPORT_CLASS(capabilities2_runner::MyRunner, capabilities2_runner::RunnerBase)
```

## Runner Execution Patterns

### Start -> Stop

The simplest runner execution pattern is to start the runner and then stop it. An example of this is the `LaunchRunner`. This pattern represents a ***self-contained*** capability. An example might be the action of a robot following another entity. This skill could be self-contained and runs continuously until stopped. This case assumes that the skill is continuously running and does not require any external triggers. ROS **Topic** subscriptions and **Launch** files are like this type.

### Start -> Trigger -> Stop

A more complex runner execution pattern is to start the runner, trigger it, and then stop it. This pattern represents a ***one-shot*** capability. An example might be the action of a robot moving to a waypoint. This skill is triggered once and then stops.

### Start -> Trigger -> Trigger -> ... -> Stop

An even more complex runner execution pattern is to start the runner, trigger it multiple times, and then stop it. This pattern represents a ***repeating*** capability. An example might be the action of a robot completing a state-machine or tree. This skill is triggered multiple times and then stops. This pattern (and the previous one) often implies that data is passed to and from the runner.

### Start -> End (no Stop)

The final runner execution pattern is to start the runner and then end it without stopping. This pattern represents a challenge for the runner API, as it is not clear when the runner should be stopped. ROS communications patterns including **Services** and **Actions** are like this type.
