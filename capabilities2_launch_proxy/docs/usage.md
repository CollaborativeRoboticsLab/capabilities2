# Using the capabilities2_launch_proxy

## Running the server

You can also start it through the package launch file:

```bash
source install/setup.bash
ros2 launch capabilities2_launch_proxy server.launch.py
```

## Sending launch goals

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	"{reference_name: package-test, package_name: capabilities2_launch_proxy, launch_file_name: package_resolved_test.launch.py}"
```

## More methods to send launch goals

### Start by shorthand package reference

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	"{launch_file_path: capabilities2_launch_proxy:package_resolved_test.launch.py}"
```

### Start by absolute path

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	"{launch_file_path: /absolute/path/to/file.launch.py}"
```

### Start with launch parameters

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 action send_goal \
	--feedback \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/action/Launch \
	'{reference_name: tb4-maze, package_name: turtlebot4_maze_sim, launch_file_name: turtlebot4_maze_small.launch.py, launch_parameters: [{key: gui, value: "false"}, {key: robot_name, value: turtlebot4}]}'
```

## Starting and stopping by service

### Start by package and launch file name

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/srv/Launch \
	"{package_name: capabilities2_launch_proxy, launch_file_name: package_resolved_test.launch.py}"
```

### Start by direct launch reference with parameters

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/launch \
	capabilities2_msgs/srv/Launch \
	'{reference_name: service-argument-test, launch_file_path: capabilities2_launch_proxy:package_resolved_argument_test.launch.py, launch_parameters: [{key: test_message, value: "hello-from-launch-service"}]}'
```

### Stop a launch by the same reference

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 service call \
	/capabilities2_launch_proxy/stop_launch \
	capabilities2_msgs/srv/Launch \
	"{package_name: capabilities2_launch_proxy, launch_file_name: package_resolved_test.launch.py}"
```