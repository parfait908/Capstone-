## Setup Instructions for robot_simple_movement Package

### Source the Workspace
Source the workspace to use the package:
```bash
source <path_to_workspace>/devel/setup.bash
```

### Running Rviz
To view the robot in Rviz, run:
```bash
roslaunch robot_simple_movement view_test.launch
```

### Running Gazebo
To simulate the robot in Gazebo, run:
```bash
roslaunch robot_simple_movement gazebo_test.launch
```
### control the robot manually in Gazebo:
```bash
rosrun controller_manager spawner diff_drive_controller
```
### Viewing the Robot and URDF Tutorial

#### Step 1: Install the URDF Tutorial Package
Download the `urdf_tutorial` package by running the following commands:
```bash
sudo apt update
sudo apt install ros-noetic-urdf-tutorial
```

#### Step 2: Run the URDF Tutorial
To view the URDF, use the following command:
```bash
roslaunch urdf_tutorial display.launch model:=/path_to_urdf/example.urdf
```
or for `.xacro` files:
```bash
roslaunch urdf_tutorial display.launch model:=/path_to_urdf/example.xacro
```

Replace `/path_to_urdf/` with the actual path to your URDF or Xacro file.




