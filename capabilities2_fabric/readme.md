# Capabilities2_fabric

Capabilities2_fabric is a ROS2 package that provides a system to coordinate and manage various capabilities as defined by the Capabilities2 framework. This package extends the functionality of the Capabilities2 package to implement a control framework based on capabilities. It is designed to parse an execution plan given via an XML file and then to identify connections between various capabilities in the system.

Currently the system support 3 types of Control fuctions `sequential`, `parallel` and `recovery`, and multitude of Event functions.

## Features

- Dynamic Capability Loading: Interacts with and manages capabilities defined by the capabilities2 framework.
- Flexible Workflow Execution: Parses XML-based plans and identifies event-driven callbacks for success, failure, or in-progress states.


## Launching capabilities2_fabric

`capabilities2_fabric/plans` folder includes sample XML plans that can be used to test the system. New plans can be added to the same folder or a different location. 

Then modify the `capabilities2_fabric/config/fabric.yaml` file to change the active execution plan.
A number of plans are availabe with the package and included in the `fabric.yaml` file that has been commented out. Uncomment them to use. Make sure to leave only one line uncommented.

```yaml
/**:
  ros__parameters:
    plan_file_path: "install/capabilities2_fabric/share/capabilities2_fabric/plans/default.xml"
    
```
Finally start the capabilities2 server. Run the following on a new terminal

```bash
source install/setup.bash
ros2 launch capabilities2_fabric fabric.launch.py
```


## XML Plan Parsing

The capabilities2_fabric package relies on XML-based plans to define workflows. These plans specify the sequence of capabilities to execute, along with the associated parameters. The XML format includes tags for capabilities as events, and control flows enabling complex workflows to be structured in a modular way.

Below is an example XML plan for configuring a set of capabilities:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<Plan>
    <Control name="sequential">
        </Control name="parallel">
            <Control name="sequential">
                <Event name="OccupancyGridRunner" provider="OccupancyGridRunner"/>
                <Event name="PromptOccupancyRunner" provider="PromptOccupancyRunner" model="llama3.1:8b"/>
            </Control>
            <Control name="sequential">
                <Event name="RobotPoseRunner" provider="RobotPoseRunner"/>
                <Event name="PromptPoseRunner" provider="PromptPoseRunner" model="llama3.1:8b"/>
            </Control>
        </Control>
        <Event name="PromptPlanRunner" provider="PromptPlanRunner" replan="false" model="llama3.1:8b"/>
        <Event name="FabricSetPlanRunner" provider="FabricSetPlanRunner"/>
    </Control>
</Plan>
```

## API

| Node |  Description |
| :---  | :---            | 
| `capabilities2_Fabric`   | Implemented the XML parsing and connection identification as well as communicating with `capabilities_server` to configure capability events |
| `capabilities2_File_Parser`   | Reads an exection plan from a given path and sends it to the `capabilities2_fabric` node. Can be used as a sample action client to work with the `capabilities2_fabric` |

| Action | Action Message | Description |
| :---  | :---            | :---        |
| `~/capabilities_fabric`           | `Plan.action`         | Receive and XML plan via the message for execution|

## Samples and Testing

### Navigation

1. [WaypointRunner Example 1](./docs/waypoint_runner_ex1.md)
Implements at the very basic fabric triggering that moves the robot from one point to another.

2. [WaypointRunner Example 2](./docs/waypoint_runner_ex2.md)
Implements navigating through 2 points using 'sequential' control functionality.


### Prompting

1. [PromptCapabilityRunner Example](./docs/prompt_capability_runner_ex1.md)
Implements requesting for robot's capabilities and prompting them to the LLM

2. [PromptOccupancyRunner Example](./docs/prompt_occupancy_runner_ex1.md)
Implements listening for robot's occupancy grid and prompting them to the LLM

2. [PromptPoseRunner Example](./docs/prompt_pose_runner_ex1.md)
Implements listening for robot's pose and prompting them to the LLM

2. [PromptPlanRunner Example](./docs/prompt_plan_runner_ex1.md)
Implements prompting the LLM for a plan for a new task and setting it to Capabilities Fabric