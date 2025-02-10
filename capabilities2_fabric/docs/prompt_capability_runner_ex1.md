## PromptCapabilityRunner Example

### Dependencies

This example uses prompt tools stack. Follow instructions from [Propmt Tools Dependency Installation](../../docs/prompt_tools_setup.md) to setup Prompt tools stack.

### Plan selection

Uncomment the  line related to `prompt_1.xml` in the `config/fabric,yaml` file

### Build the package to apply changes

In the workspace root run,

```bash
colcon build
```

### Start the Prompt Tools stack

```bash
source install/setup.bash
ros2 launch prompt_bridge prompt_bridge.launch.py
```

### Start the Capabilities2 Server

```bash
source install/setup.bash
ros2 launch capabilities2_server server.launch.py
```

### Start the Capabilities2 Fabric

```bash
source install/setup.bash
ros2 launch capabilities2_fabric fabric.launch.py
```