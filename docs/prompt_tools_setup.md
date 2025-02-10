# Dependency installation for Prompt Tools Runners

## Clone packages

Clone the prompt tools package same workspace if its not already availabe in the workspace. Capabilities2 Prompt Runners are dependent on this package.

```bash
cd src
git clone https://github.com/CollaborativeRoboticsLab/prompt_tools.git -b develop
```

## Dependency Installation

Move to workspace root and run the following command to install dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```