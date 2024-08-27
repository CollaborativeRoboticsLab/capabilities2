# capabilities2_launch_proxy

provide an api proxy to ros2 launch. Implements features that are only available in python as launch api is not available in C++.

The essential features to implement the ros1 `capabilities` package launch functionality in ros2.

Use in conjunction with `capabilities2_server` package, to provide a full capabilities server.

## how it works

implements three main functions:

- `launch` - run a launch file
- `shutdown` - shutdown a launch
- `events` - send events

the lifecylce of these functions are typically handled by the capabilities server but could be used for other things.
