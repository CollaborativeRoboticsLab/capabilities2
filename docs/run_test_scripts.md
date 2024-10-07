# Run test scripts

Capabilities features can be tested using the provided test scripts. The test scripts primarily test the capabilities2 server using service clients. Launch the capabilities2 server before running the test scripts [information here](../capabilities2_server/README.md). The test scripts are written to test against the `std_capabilities` package. Make sure that the `std_capabilities` package is built and sourced before running the test scripts.

> **Note**: The test scripts use the `bondby` package. Make sure to install the `bondpy` package before running the test scripts.

Run the tests with python3. The test scripts are located in the `capabilities2_server/test` directory. Make sure to source the workspace before running the test.

```bash
# example
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

There is another test script in the `capabilities2_launch_proxy` package that tests using a capability.

| Test Script | Description |
| --- | --- |
| [call_use_launch_runner.py](../capabilities2_launch_proxy/test/call_use_launch_runner.py) | Test using a launch runner based capability. This tests the bond, use, and get running features of the capabilities server |
