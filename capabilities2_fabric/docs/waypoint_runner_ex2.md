## WaypointRunner Example 2

### Dependencies

This example uses nav2 stack and turtlebot3. Follow instructions from [Nav2 Dependency Installation](../../docs/nav2_setup.md) to setup nav stack.

### Plan selection

Uncomment the  line related to `navigation_2.xml` in the `config/fabric,yaml` file

### Build the package to apply changes

In the workspace root run,

```bash
colcon build
```

### Start the turtlebot simulation

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Start the Navigation2 stack

```bash
source install/setup.bash
ros2 launch nav_stack system.launch.py
```

### Start the Capabilities2 Server

```bash
source install/setup.bash
ros2 launch capabilities2_server server.launch.py
```

### Start the fabric

```bash
source install/setup.bash
ros2 launch capabilities2_fabric fabric.launch.py
```