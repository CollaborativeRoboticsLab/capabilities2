## Using Capabilities package (Development and Deployment)

The current package has been tested on Ubuntu and ROS2 Humble, Rolling and Jazzy. Complete following sections in order.

### Base folder creation

Create the workspace folder by running following commands in the terminal.

```bash
mkdir -p /home/$USER/capabilities_ws/src
cd /home/$USER/capabilities_ws/src
```

### Cloning the Package

Clone the package using Git

```bash
git clone -b capabilities2-server-fabric https://github.com/CollaborativeRoboticsLab/capabilities2.git
git clone -b develop https://github.com/CollaborativeRoboticsLab/std_capabilities.git
git clone https://github.com/AIResearchLab/nav_stack.git
```

### Devcontainer

A `devcontainer` is provided for developing the capabilities2 meta-package. The container can be used with [Microsoft Visual Studio Code](https://code.visualstudio.com/) and [Remote Development Extention](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack). 

If there are multiple packages in the workspace as with the usual case, move the `.devcontainer` folder into the parent folder such that it is located as follows,
```txt
colcon_ws
--> build
--> install
--> log
--> src
    --> .devcontainer
    --> capabilities2
    --> pakcage ..
```

Close and reopen the vs code editor so that devcontainer is automatically detected. After that, Dependencies need to be installed in the container. Install the dependencies with rosdep:

```bash
# in the devcontainer
rosdep install --from-paths src --ignore-src -r -y
```

### Compile

Use colcon to build the packages:

```bash
colcon build
```
