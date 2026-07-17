# Run test scripts

Capabilities features can be tested using the provided test scripts. The test scripts primarily test the capabilities2 server using service clients. Launch the capabilities2 server before running the test scripts using

```bash
source install/setup.bash
ros2 launch capabilities2_server capabilities2_server.launch.py
```

The test scripts are written to test against the `std_capabilities` package. Make sure that the `std_capabilities` package is copied along with the `capabilities2` package to the `capabilities2_ws/src` folder and is built and sourced before running the test scripts.

> **Note**: The test scripts use the `bondpy` package. Make sure it is installed before running the test scripts. \
> `sudo apt install ros-$ROS_DISTRO-bondpy`

Run the tests with python3. The test scripts are located in the `capabilities2_server/test` directory. Make sure to source the workspace before running the test.

```bash
# example
source install/setup.bash
cd src/capabilities2/capabilities2_server/test
python3 call_establish_bond.py
```

| Test Script | Description |
| --- | --- |
| [call_establish_bond.py](../capabilities2_server/test/call_establish_bond.py) | Test establishing a bond. `ctrl-C` to kill the bond |
| [call_get_interfaces.py](../capabilities2_server/test/call_get_interfaces.py) | Test getting all loaded interfaces |
| [call_get_providers.py](../capabilities2_server/test/call_get_providers.py) | Test getting loaded providers for the `empty` interface |
| [call_get_semantic_interfaces.py](../capabilities2_server/test/call_get_semantic_interfaces.py) | Get loaded semantic interfaces |
| [call_get_specs.py](../capabilities2_server/test/call_get_specs.py) | Get all the loaded spec files |
| [call_register_cap.py](../capabilities2_server/test/call_register_cap.py) | Register a new specification file |

Launch-runner-specific tests are currently not provided. The `LaunchRunner` header and plugin entry remain in the codebase, but launch support is intentionally deferred until a replacement approach for the current ROS2 launch system is ready.
