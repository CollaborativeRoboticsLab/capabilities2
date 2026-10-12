# Capabilies2 Launch Proxy and Launch Runner

## Capabilities2 Launch Proxy

The launch proxy package provides the long-lived ROS 2 action server that actually starts, monitors, and stops launch files. It owns the persistent `launch.LaunchService`, tracks child processes, and exposes launch lifecycle through the `capabilities2_msgs/action/Launch` action.

In the current design, launch-backed capability providers (capabilities2_runner::LaunchRunner) do not execute launch files directly inside the capabilities server process. Instead, they delegate launch execution to this action server.

## Launch Runner

`capabilities2_runner::LaunchRunner` is the bridge between capability providers that declare a launch-style `runner:` field and the launch proxy action server.

When the capabilities server starts a provider whose `runner` value looks like a launch file reference such as `package_name/file.launch.py`, `RunnerCache` selects `LaunchRunner`. `LaunchRunner` then sends a `capabilities2_msgs/action/Launch` goal to the launch proxy action server instead of spawning the launch itself.

### Why a dedicated client node exists

`LaunchRunner` creates its own internal ROS node and spins that node on a dedicated single-threaded executor in a separate thread.

This is important because the capabilities server is already running on its own executor and may be inside a service callback when a capability is started. If `LaunchRunner` used the capabilities server node directly and then blocked waiting for the action goal to be accepted, the action client's callbacks could be starved by the same executor thread that is currently busy handling the capability request.

That creates a failure mode where:

1. the capabilities server enters `start_capability`
2. `LaunchRunner::start()` sends a goal to the launch proxy action server
3. `LaunchRunner::start()` waits for goal acceptance
4. the callback needed to complete goal acceptance cannot run promptly because the same executor context is still occupied
5. launch startup appears to hang or time out even though the action server is alive

To avoid that coupling, `LaunchRunner` now creates:

- an internal action-client node
- a dedicated single-threaded executor for that node
- a dedicated executor thread that spins the client node independently

This separation allows the launch action client to receive goal responses, feedback, cancellation responses, and results without depending on the capabilities server executor becoming idle first.

### Lifecycle behavior

The launch runner lifecycle is now:

1. wait for the launch proxy action server
2. send a `Launch` goal containing the provider's launch reference
3. emit `STARTED` once the goal is accepted
4. emit `LAUNCHED` once launch feedback shows a live launched process
5. forward launch failures into runner failure events
6. cancel the action goal on runner stop
7. emit `STOPPED` when cancellation or launch completion is observed

### Why this design matters

This arrangement keeps responsibilities separate:

- the capabilities server remains responsible for capability orchestration and dependency management
- `LaunchRunner` remains responsible for runner lifecycle and event emission
- the launch proxy remains responsible for launch execution, process tracking, feedback, and cancellation

That makes launch-backed capabilities behave like other capability runners while still using ROS 2 launch through a single persistent launch runtime.