# Basic Tutorial

This tutorial will guide you through the basic steps of using `capabilities2`. It will cover the following topics:

1. [Creating a capability](#creating-a-capability) using yaml model files.
2. [Run the capabilities server](#run-the-capabilities-server) to serve the capability.
3. [Run the capabilities runner](#run-the-capabilities-runner) to execute the capability.

The tutorial will create a simple capability that prints "Hello, World!" to the console, using talker-listener nodes as a backend sub-system.

The capabilities server will serve the capability, and then spawn a runner that will execute the capability.

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

Create a C++ class that implements the provider. This class will be used to run the capability. Create a new file `talker.cpp` in the `src` directory of the `hello_capability_world` package. In this example, the capability publishes immediately when started, so the class can inherit from `capabilities2_runner::NoTriggerRunner` and be exported as a plugin.

```cpp
// talker.cpp
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "capabilities2_runner/notrigger_runner.hpp"
#include "std_msgs/msg/string.hpp"

namespace hello_capability_world
{
class Talker : public capabilities2_runner::NoTriggerRunner
{
public:
  Talker() = default;

  void start(
    rclcpp::Node::SharedPtr node,
    const capabilities2_runner::runner_opts& run_config,
    const std::string& bond_id) override
    {
    // init base class gets node info and stores the runner configuration
    init_base(node, run_config);

        // create a publisher to the chatter topic
        auto chatter_pub = node_->create_publisher<std_msgs::msg::String>("chatter", 10);

    emit_started(bond_id, "", param_on_started());

        // send a message to the chatter topic
    std_msgs::msg::String msg;
    msg.data = "Hello, World!";
    chatter_pub->publish(msg);
    }

  void stop(const std::string& bond_id, const std::string& instance_id = "") override
  {
    emit_stopped(bond_id, instance_id, param_on_stopped());
    }
};
} // namespace hello_capability_world

// export the plugin
PLUGINLIB_EXPORT_CLASS(hello_capability_world::Talker, capabilities2_runner::RunnerBase)
```

#### Export the plugin in the CMakeLists.txt

Follow the ROS2 pluginlib pattern to export the runner plugin.

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
add_library(${PROJECT_NAME} SHARED src/talker.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  pluginlib
  capabilities2_runner
  std_msgs
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
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

Run the capabilities server node and provide the config file with ROS parameters.

```bash
# run the capabilities server node with the config file
ros2 run capabilities2_server capabilities2_server_node --ros-args --params-file /path/to/hello_capability_world/config/hello_capability_world.yaml
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
