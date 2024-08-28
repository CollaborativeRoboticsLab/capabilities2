# capabilities2_launch_proxy

provides an api proxy to ros2 launch. Implements features that are only available in python as launch api is not available in C++.

Contains the essential features to implement the ros1 [`capabilities`](https://wiki.ros.org/capabilities) package launch functionality in ros2.

Use in conjunction with `capabilities2_server` package, to provide a full capabilities server.

## how it works

implements three main functions:

- `launch` - run a launch file
- `shutdown` - shutdown a launch
- `events` - send events

the lifecylce of these functions are typically handled by the capabilities server but could be used for other things.

## use without capabilities server

The proxy provides an action called `~/launch`. The goal is a file path to launch from. The feedback provides events about the launch status.
