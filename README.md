## Setup Instructions for My Robot Description Package

### Build the Package
Make sure to build the package using `catkin_make`:
```bash
catkin_make
```

### Source the Workspace
Source the workspace to use the package:
```bash
source <path_to_workspace>/devel/setup.bash
```

### Running Rviz
To view the robot in Rviz, run:
```bash
roslaunch my_robot_description display.launch
```

### Running Gazebo
To simulate the robot in Gazebo, run:
```bash
roslaunch my_robot_description gazebo.launch
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




