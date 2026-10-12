# Example with TurtleBot4 simulation

If the workspace has `turtlebot4_maze_sim` from [CollaborativeRoboticsLab/turtlebot4_simulations](https://github.com/CollaborativeRoboticsLab/turtlebot4_simulations) built and sourced, you can start simulation-related launches through either the action server or the compatibility launch services. Otherwise clone and build that repository first.

### Start the server

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch capabilities2_launch_proxy server.launch.py
```

## Action-based launch goals

### Start the simulation world

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	'{reference_name: tb4-maze, package_name: turtlebot4_maze_sim, launch_file_name: turtlebot4_maze_small.launch.py, launch_parameters: [{key: gui, value: "false"}]}'
```

### Start the navigation stack

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	"{package_name: turtlebot4_maze_sim, launch_file_name: sim_navigation.launch.py}"
```

## Service-based launch goals

### Start the simulation world

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/srv/Launch \
	'{reference_name: tb4-maze, package_name: turtlebot4_maze_sim, launch_file_name: turtlebot4_maze_small.launch.py, launch_parameters: [{key: gui, value: "false"}]}'
```

### Start the navigation stack

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/srv/Launch \
	"{package_name: turtlebot4_maze_sim, launch_file_name: sim_navigation.launch.py}"
```

### Stop the simulation world

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/stop_launch \
	capabilities2_msgs/srv/Launch \
	'{package_name: turtlebot4_maze_sim, launch_file_name: turtlebot4_maze_small.launch.py}'
```

### Stop the navigation stack

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/stop_launch \
	capabilities2_msgs/srv/Launch \
	"{package_name: turtlebot4_maze_sim, launch_file_name: sim_navigation.launch.py}"
```	
