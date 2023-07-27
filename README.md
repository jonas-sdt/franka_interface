# Franka Interface

## Description

This repository contains a custom ROS interface for the Franka Emika Panda robot. It is based on the moveit framework, but provides a more convenient interface for the robot and a custom parameterization.

## Library Documentation

[franka_interface::FrankaInterface class reference](franka_interface/doc/md/Classes/classfranka__interface_1_1FrankaInterface.md)

[franka_interface::PlanningFailed](franka_interface/doc/md/Classes/classfranka__interface_1_1PlanningFailed.md)

[franka_interface::ExecutionFailed](franka_interface/doc/md/Classes/classfranka__interface_1_1ExecutionFailed.md)

[franka_interface namespace reference](franka_interface/doc/md/Namespaces/namespacefranka__interface.md)

## Usage

### Installation

This ROS package depends on the following ROS packages:

- `rviz_visual_tools`
- `moveit_visual_tool`
- `moveit`
- `franka_ros`

To install the dependencies run the following command:

```bash
sudo apt install ros-noetic-rviz-visual-tools ros-noetic-moveit-visual-tools ros-noetic-moveit ros-noetic-franka-ros
```

### Creating a project

Each project should be created in a separate ROS package. This package needs to depend on the `franka_interface` package.

### Writing a program

Take a look at the [template program](franka_test_pkg/src/template_node.cpp). It contains a simple example of how to use the interface.

### Running a program

To run your program you have to either start the simulation stack or the franka control stack environment. To start the simulation stack run the following command:

```bash
roslaunch franka_interface sim.launch
```

To start the franka control stack run the following command:

```bash
roslaunch franka_interface franka.launch
```

To start the franka control stack with effort control run the following command:

```bash
roslaunch franka_interface franka.launch transmission:=effort
```

Afterwards you can run your program.
