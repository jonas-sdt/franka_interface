---
title: Franka Interface

---

# Franka Interface




# Description

This repository contains a custom ROS interface for the Franka Emika Panda robot. It is based on the moveit framework, but provides a more convenient interface for the robot and a custom parameterization.


# Installation


## Dependencies



* [ROS 1 noetic (other ROS 1 version might also work but were not tested.)](http://wiki.ros.org)
* [MoveIt! 1](https://moveit.ros.org/)


## Optional:

This repository contains convenience scripts for the use with Visual Studio Code. It is recommended to start the repository from within VS Code. This requires the following extensions:



* [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
* [ROS](https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros)


## Building



1. Clone the repository into your catkin workspace
2. Build the workspace with `catkin_make`


# Starting the system

The first step before you start your custom applications is to start the RVIZ UI, MoveIt! and optionally the control interface to the real robot.


## Simulated robot

The robot can be launched with the following command:

```bash

roslaunch franka_interface sim.launch
```


## Real robot

The robot can be launched with the following command:

```bash

roslaunch franka_interface franka.launch
```


# Writing custom applications

To use the franka interface library in your own applications, you need to add it as a dependency. [Example CMakeLists.txt](franka_test_pkg/CMakeLists.txt) and a [example package.xml](franka_test_pkg/package.xml) is provided in the `franka_test_pkg` package.

TL;DR you need to add the following lines to your CMakeLists.txt:

```cmake

find_package(franka_interface REQUIRED)
target_link_libraries(${PROJECT_NAME} franka_interface::franka_interface)
```

And the following line to your package.xml:

```xml

<build_depend>franka_interface</build_depend>
```


# Debugging with VS Code



1. Start the RVIZ UI as described above
2. Create the following VS Code launch configuration in `.vscode/launch.json`:
```json { "version": "0.2.0", "configurations": [ { "name": "ROS Launch", "type": "ros", "request": "launch", "target": "<path to your launch file>", "internalConsoleOptions": "openOnSessionStart", "preLaunchTask": "catkin_make debug" } ] } ```
To be able to debug a ROS node it needs to be started by a ROS launch file. Make sure to replace `<path to your launch file>` with the path to the launch file you want to debug.
3. Create the following VS Code build configuration in `.vscode/tasks.json`:
```json { "version": "2.0.0", "tasks": [ { "label": "catkin_make debug", "type": "shell", "command": "catkin_make -DCMAKE_BUILD_TYPE=Debug", "group": { "kind": "build", "isDefault": true }, "problemMatcher": [], "runOptions": {"instanceLimit": 1}, "icon": {"color": "terminal.ansiBlue", "id": "wrench"} } ] } ```
4. Start the launch configuration form step 2

You can find examples of this configuration here:



* launch.json: [franka_interface/.vscode/launch.json](franka_interface/.vscode/launch.json)
* tasks.json: [franka_interface/.vscode/tasks.json](franka_interface/.vscode/tasks.json)


# Library Documentation

[franka_interface class](franka_interface/doc/html/classfranka__interface_1_1FrankaInterface.html)

-------------------------------

Updated on 2023-07-07 at 13:14:33 +0200
