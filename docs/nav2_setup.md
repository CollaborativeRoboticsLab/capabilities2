# Dependency installation for Nav2 Runners

## Install nav2 stack

```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
```

## Clone configuration

Clone the nav_stack default configuration and launch files to the same workspace if its not already availabe in the workspace. Capabilities2 Nav2 Runners are dependent on this package.

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/nav_stack.git
```

## Turtlebot3 Simulation (Optional) 
If using a simulated turtlebot3 for testing, install using following commands.

```bash
sudo apt install ros-humble-turtlebot3*
```