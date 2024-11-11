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
git clone -b develop https://github.com/CollaborativeRoboticsLab/capabilities2.git
```

### Basic configuration

On the terminal run the following command to identify the $USER and note down the value

```bash
echo $USER
```

Then open the `/home/$USER/capabilities_ws/src/capabilities2/capabilities2_server/ config/capabilities.yaml` with an available text editor. Either gedit or nano can be used.

```sh
nano /home/$USER/capabilities_ws/src/capabilities2/capabilities2_server/config/capabilities.yaml
```

In the opened file, replace $USER value in lines 9 & 10 with above identified value for $USER. if the $USER is ubuntu, those lines should be

```yaml
      - /home/ubuntu/capabilities_ws/src
      - /home/ubuntu/capabilities_ws/src/capabilities2
```

Save the file and close.

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