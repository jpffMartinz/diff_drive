# Diff Drive Robot Package

A ROS2 package for a differential drive robot with visualization and simulation capabilities.

## Prerequisites

- ROS2 (Jazzy)
- colcon build tools
- RViz2

## Quick Start - Viewing the Robot in RViz

Follow these steps to visualize the robot model:

### 1. Build the Package

Navigate to your ROS2 workspace root directory:

```cd ~/ros2_ws```



### 2. Source the Environment

Source the setup file to make the package available:

```source install/setup.bash```


### 3. Launch RViz Visualization

Run the display launch file to open RViz with the robot model:

```ros2 launch diff_drive display.launch.py```



This will:
- Load the robot's URDF model
- Start RViz2 with a pre-configured view
- Display the robot with its joints and links

## Viewing the Robot in Gazebo
in construction

## Package Contents

- **`launch/`** - Launch files for different functionalities
- **`urdf/`** - Robot description files (URDF/Xacro)
- **`rviz/`** - RViz configuration files
- **`meshes/`** - 3D mesh files for robot visualization
- **`worlds/`** - Gazebo world files for simulation
