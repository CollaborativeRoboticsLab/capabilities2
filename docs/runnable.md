# Runnable Information

This provides information about the runnable capabilities in the system. The `get_runnable_specs` service returns runnable specifications for invokable interface and provider pairs. Because ROS service responses do not expose a map type here, the response uses two parallel arrays:

- `index_of_specs`: keys in the form `interface_name|provider_name`
- `runnable_specs`: matching `RunnableSpec` messages containing the runnable YAML string
            
```bash
source install/setup.bash
ros2 launch capabilities2_server capabilities2_server.launch.py
```

```bash
source install/setup.bash
ros2 service call /capabilities/get_runnable_specs capabilities2_msgs/srv/GetRunnableSpecs
```