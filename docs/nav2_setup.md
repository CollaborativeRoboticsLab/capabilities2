# Dependency installation for Nav2 Runners

## Install nav2 stack and slam-toolbox

Run the following commands to install nav2 stack

```bash
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
```

## Clone Nav2 configuration

Clone the nav_stack default configuration and launch files to the same workspace if its not already availabe in the workspace. Capabilities2 Nav2 Runners are dependent on this package.

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/nav_stack.git
```

## Clone Capabilities2 plugin for Nav2 stack

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/capabilities2_runner_nav2.git
```

## Turtlebot3 Simulation (Optional) 

If using a simulated turtlebot3 for testing, install using following commands.

```bash
sudo apt install ros-$ROS_DISTRO-turtlebot3*
```

## Dependency Installation

Move to workspace root and run the following command to install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```