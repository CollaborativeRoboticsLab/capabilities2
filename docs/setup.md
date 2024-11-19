## Using Capabilities package (Development and Deployment)

The current package has been tested on Ubuntu and ROS2 Humble, Rolling and Jazzy. Complete following sections in order.

### Base folder creation

Create the workspace folder by running following commands in the terminal.

```bash
mkdir -p /home/$USER/capabilities_ws/src
cd /home/$USER/capabilities_ws/src
```

### Cloning the Packages

Clone the package using Git

```bash
git clone -b capabilities2-server-fabric https://github.com/CollaborativeRoboticsLab/capabilities2.git
git clone -b develop https://github.com/CollaborativeRoboticsLab/std_capabilities.git
git clone https://github.com/AIResearchLab/nav_stack.git
```

### Dependency installation

Move the terminal to workspace root and install dependencies.

```bash
cd /home/$USER/capabilities_ws
```
```bash
rosdep install --from-paths src --ignore-src -r -y
```

### Compile

Use colcon to build the packages:

```bash
colcon build
```
