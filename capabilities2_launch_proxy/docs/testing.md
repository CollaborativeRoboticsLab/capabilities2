# Testing the capabilities2_launch_proxy package

The `capabilities2_launch_proxy` package includes a suite of tests to ensure its functionality and reliability. These tests cover various aspects of the package's behavior, including launch file resolution, parameter passing, process management, and action handling.

## Integration Tests

The primary integration tests are located in `test/test_capabilities2_launch_integration.py`. These tests verify the package's ability to:

- Resolve launch files by package name and file name
- Resolve launch files by shorthand package reference syntax
- Pass launch parameters correctly
- Handle action cancellation
- Forward logs to the appropriate destinations
- Optionally mirror child logs through the server ROS logger
- Publish resource information to the server-wide resource topic

## Running the Tests

To run the tests, use the following command:

```bash
colcon test --packages-select capabilities2_launch_proxy
colcon test-result --all
```

This will execute all tests for the `capabilities2_launch_proxy` package and display the results.