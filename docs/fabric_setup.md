# Dependency installation for Prompt Tools Runners

## Clone packages

Clone the fabric package same workspace if its not already availabe in the workspace. Capabilities2 Fabric Runners are dependent on this package.

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/fabric.git
```

## Clone Capabilities2 plugin for Fabric stack

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/fabric.git
```

## Dependency Installation

Move to workspace root and run the following command to install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```