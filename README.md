# RSX Arm ROS2 Migration (Jazzy) - 2025/26

Welcome to `arm-ros2`! This repository will host the ROS 2 (Jazzy) version of the RSX arm control software, migrated from the `arm/` folder in the old ROS 1 `rsx-rover` repo. Proceed with caution.

## 🚀 Prerequisites

Make sure you have the following:

- **Ubuntu 24.04** 
- **ROS 2 Jazzy** fully installed and sourced (https://docs.ros.org/en/jazzy/Installation.html)

Get the essential tools: 
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
```

Initialize rosdep (once per machine):
```bash
sudo rosdep init
rosdep update
```
## Game Plan

## Workspace Setup

1. Create a new workspace
```bash
mkdir -p ~/arm_ros2_ws/src
cd ~/arm_ros2_ws
```
2. Clone this repository
```bash
cd src
git clone https://github.com/rsx-utoronto/arm-ros2.git
```

3. Install dependencies listed in package.xml
```bash
cd ~/arm_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```
4. Set up a Python Virtual Environment
We'll isolate all Python dependencies (outside ROS) in a virtual environment.
```bash
sudo apt install python3-venv
python3 -m venv arm_env
source arm_env/bin/activate
```
Then add this to your `.bashrc`
```bash
alias workon_arm="cd ~/arm_ros2_ws && source arm_env/bin/activate && source install/setup.bash"
```

5. Build the workspace
```bash
colcon build --symlink-install
```

6. Source the setup files
```bash
source install/setup.bash
```

### Note: Remember to Track Python Dependencies!!!

Create a requirements.txt in the repo root that looks like this: 

```
numpy
scipy
geometry_msgs
rclpy
```
And then, add any dependencies you discover as you port the code. Install them inside the venv with:

```
pip install -r requirements.txt
```
## Proposed Structure 
```
arm-ros2/
├── arm_ros2/              # ROS 2 Python Package
│   ├── arm_controller.py  # <- migrated version
│   ├── __init__.py
├── launch/
│   ├── arm.launch.py      # <- migrated launch file
├── msg/
│   ├── ArmStatus.msg      # <- if used
├── srv/
│   ├── ArmIK.srv          # <- if used
├── setup.py
├── setup.cfg
├── package.xml
├── requirements.txt

```

## Proposed Migration Workflow

1. Analyze the Original `arm/` Folder
- Identify ROS 1 Python nodes using `rospy`.
- Note topics, services, parameters, dependencies, and launch files.
- Document dependencies in `package.xml`. 

2. Set Up ROS 2 Package
- Rename package (e.g., `arm_ros2`) and update metadata in package.xml and setup.py.
- Add necessary ROS 2 dependencies: `rclpy`, `std_msgs`, etc.

3. Migrate Nodes Incrementally
For each ROS 1 node:
- Create ROS 2 Python version using rclpy and ROS 2 APIs.
- Add entry point in setup.py:
```python
entry_points={
    'console_scripts': [
        'arm_controller = arm_ros2.arm_controller:main',
    ],
},

```
- Build, source, and test with:
```bash
cd ~/arm_ros2_ws
colcon build
source install/setup.bash
ros2 run arm_ros2 arm_controller
```

4. Migrate Launch Files
Convert .launch to .launch.py, using ROS 2 conventions:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='arm_ros2', executable='mynode', name='mynode'),
    ])
```

5. Add Custom Messages/Services/Actions
If migrating custom `msg/`, `srv/`, or `action/` files:
- Place definitions in the respective ROS 2 directories.
- Update `package.xml` and `CMakeLists.txt`.
- Rebuild and import accordingly in nodes.

## Team Workflow
- Work only in this repository (`arm-ros2`); keep `rsx-rover` as a reference.
- Use feature branches for each node migration.
- Submit PRs and conduct peer reviews before merging.
- After migrating each node:
    - Build:
      ```bash
      colcon build
      ```
    - Source:
      ```bash
      source install/setup.bash
      ```
    - Run & test:
      ```bash
      ros2 run arm_ros2 <node>
      ros2 topic echo /<topic>
      ```
- Track progress with an issue board (e.g., "Migrate node X", "Convert launch file Y").

# Collaboration Code of Conduct:

  1. Engineering Specification and Background Research (FOCs, Research into task)
  2. Candidate Designs
      - Don’t need a final design, block diagrams with explanation are fine
  3. Final Design
      - Can be one of the previous two designs or something all new based on learned
      - Can be a block diagram with research and explanation

  Example: (https://docs.google.com/document/d/1crGv72SZPlyTfEEYrCo4EamMKkqWRGyadf1QlVXck3k/edit?tab=t.0)




