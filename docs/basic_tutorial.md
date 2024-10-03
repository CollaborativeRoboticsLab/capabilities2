# Basic Tutorial

This tutorial will guide you through the basic steps of using the `capabiliites2`. It will cover the following topics:

1. [Creating a capability](#creating-a-capability) using yaml model files.
2. [Run the capabilities server](#run-the-capabilities-server) to serve the capability.
3. [Run the capabilities runner](#run-the-capabilities-runner) to execute the capability.

The tutorial will create a simple capability that prints "Hello, World!" to the console, using talker-listener nodes as a backend sub-system.

The capabilities server will serve the capability, and the spawn a runner that will execute the capability.

## Creating a capability

First, create a package to hold the capability. In this tutorial, we will create a package called `hello_capability_world`.

```bash
ros2 pkg create --build-type ament_cmake hello_capability_world
```

Next, create capability specification files in the package. These files will define the capability and the provider. Create 2 yaml files, `talker_interface.yaml` and `talker_provider.yaml` in the `hello_capability_world` package.

### Spec files (.yaml)

#### talker_interface.yaml

```yaml
# talker_interface.yaml
name: talker
spec_type: interface
spec_version: 1.1
description: A simple talker capability that prints "Hello, World!" to the console.
interface:
  topics:
    "chatter":
      type: std_msgs/msg/String
      description: A topic that prints "Hello, World!" to the console.
```

#### talker_provider.yaml

```yaml
# talker_provider.yaml
name: talker
spec_type: provider
spec_version: 1.1
description: A simple talker provider that prints "Hello, World!" to the console.
implements: hello_capability_world/talker
runner: hello_capability_world::Talker # this is the C++ class that implements the provider
```

### Export the capability in the package.xml

Add the following to the `package.xml` file in the `hello_capability_world` package.

```xml
<!-- within the export tag -->
<export>
    <!-- add the capability files -->
    <capability_interface>talker_interface.yaml</capability_interface>
    <capability_provider>talker_provider.yaml</capability_provider>
</export>
```

### Create a runner plugin

Create a C++ class that implements the provider. This class will be used to run the capability. Create a new file `talker.cpp` in the `src` directory of the `hello_capability_world` package. The class should inherit from the `capabilities2_runner::RunnerBase` class, and should be exorted as a plugin.

```cpp
// talker.cpp
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "capabilities2_runner/runner_base.hpp"
#include "std_msgs/msg/string.hpp"

namespace hello_capability_world
{
class Talker : public capabilities2_runner::RunnerBase
{
public:
    Talker() = default;

    // implement the start method
    virtual void start(rclcpp::Node::SharedPtr node, const runner_opts& run_config,
                        std::function<void(const std::string&)> on_started = nullptr,
                        std::function<void(const std::string&)> on_terminated = nullptr,
                        std::function<void(const std::string&)> on_stopped = nullptr)
    {
        // init base class gets node info and sets up the event handlers
        init_base(node, run_config, on_started, on_terminated, on_stopped);

        // create a publisher to the chatter topic
        // can now use the internal node (node_) to create the publisher since it was set up in the base class
        // the 'chatter' topic is defined in the talker_interface.yaml file so it could be collected programmatically
        // the topic name is passed in the run_config
        // in this example, we are hardcoding the topic name
        auto chatter_pub = node_->create_publisher<std_msgs::msg::String>("chatter", 10);

        // send a starting runner event
        if (on_started)
        {
            on_started(get_interface());
        }

        // send a message to the chatter topic
        chatter_pub->publish(std_msgs::msg::String().set_data("Hello, World!"));

        // send a terminated runner event
        if (on_terminated)
        {
            on_terminated(get_interface());
        }
    }

    // the other virtual methods are not implemented in this example
    // the stop method is used to stop the capability
    // this method is not implemented in this example but it does emit a stopped event
    virtual void stop() {
        // stop the runner event
        if (on_stopped)
        {
            on_stopped(get_interface());
        }
    }

    // the trigger method is used to trigger the capability with parameters after it has been started
    // this method is not implemented in this example
    virtual std::optional<std::function<void(std::shared_ptr<tinyxml2::XMLElement>)>>
    trigger(std::shared_ptr<tinyxml2::XMLElement> parameters = nullptr) {}
};
} // namespace hello_capability_world

// export the plugin
PLUGINLIB_EXPORT_CLASS(hello_capability_world::Talker, capabilities2_runner::RunnerBase)
```

#### Export the plugin in the CMakeLists.txt

Follow the [creating a plugin]() tutorial from ROS2 to export a plugin.

```xml
<!-- plugins.xml -->
<library path="lib">
  <class name="hello_capability_world::Talker" type="hello_capability_world::Talker" base_class_type="capabilities2_runner::RunnerBase">
    <description>A simple talker provider that prints "Hello, World!" to the console.</description>
    </class>
</library>
```

```cmake
# CMakeLists.txt
ament_export_interfaces(export_hello_capability_world HAS_LIBRARY_TARGET)
```

## Run the capabilities server

The capabilities server is the main node in the capabilities2 package. It is responsible for managing capabilities, providers, and semantic interfaces. It is also responsible for starting and stopping capabilities.

### Create a new config file

Create a new config file in the `hello_capability_world` package. This file will define the capabilities that the server will serve.

```yaml
# config/hello_capability_world.yaml
/**:
  ros__parameters:
    loop_rate: 5.0 # Hz
    db_file: ~/.ros/capabilities/capabilities.sqlite3
    rebuild: false # Set to true to rebuild the database
    package_paths: # paths to search for capabilities
      - /opt/ros/jazzy/share # default ROS2 package paths for Jazzy
      - /home/ubuntu/colcon_ws/src
      - /home/ubuntu/colcon_ws/src/capabilities2 # absolute paths for now, these paths assume the capabilities2 package is in the colcon workspace of the ubuntu user (devcontainer user)
```

Launch the capabilities server using the launch file and pass the config file as an argument.

```bash
# run the capabilities server using the launch file
ros2 launch capabilities2_server capabilities2_server.launch.py config_file:=/path/to/hello_capability_world/config/hello_capability_world.yaml
```

We also need to run the talker-listener nodes as well. The talker-listener nodes are provided by the `demo_nodes_cpp` package. We can run the talker-listener nodes using the following command:

```bash
# in a new terminal
# run the listener node
ros2 run demo_nodes_cpp listener
```

## Run the capabilities runner

Now that the capabilities server is running, we can run the capabilities runner to execute the capability.

```bash
# in a new terminal
# run the capabilities runner using a service call
ros2 service call /capabilities/start_capability capabilities2_msgs/srv/StartCapability "{capability: 'hello_capability_world/talker', provider: 'hello_capability_world/talker'}"
```

The capabilities runner will start the capability and print "Hello, World!" to the console of the listener node.
