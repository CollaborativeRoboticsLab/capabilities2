## WaypointRunner Example 1

### Dependencies

This example uses nav2 stack and turtlebot3. 

Run the following commands to install nav2 stack

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

Run the following commands to install turtlebot3

```bash
sudo apt install ros-humble-turtlebot3*
```

Clone the nav_stack default configuration and launch files to the same workspace if its not already availabe in the workspace. Capabilities2 Nav2 Runners are dependent on this package.

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/nav_stack.git
```

### Plan selection

Uncomment the  line related to `navigation_1.xml` in the `config/fabric,yaml` file

### Build the package to apply changes

In the workspace root run,

```bash
colcon build
```

### Start the turtlebot simulation

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
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