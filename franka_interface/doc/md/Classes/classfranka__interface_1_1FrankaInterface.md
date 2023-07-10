---
title: franka_interface::FrankaInterface

---

# franka_interface::FrankaInterface






`#include <franka_interface.hpp>`

## Public Functions

|                | Name           |
| -------------- | -------------- |
| void | **[activate_collision_check](Classes/classfranka__interface_1_1FrankaInterface.md#function-activate-collision-check)**()<br>Activates the collision check for the planning scene.  |
| void | **[add_collision_object](Classes/classfranka__interface_1_1FrankaInterface.md#function-add-collision-object)**(moveit_msgs::CollisionObject collision_object)<br>Add a collision object to the planning scene.  |
| void | **[close_gripper](Classes/classfranka__interface_1_1FrankaInterface.md#function-close-gripper)**()<br>Closes the gripper completely.  |
| void | **[deactivate_collision_check](Classes/classfranka__interface_1_1FrankaInterface.md#function-deactivate-collision-check)**()<br>Deactivates the collision check for the planning scene.  |
| | **[FrankaInterface](Classes/classfranka__interface_1_1FrankaInterface.md#function-frankainterface)**(ros::NodeHandle & nh, std::string robot_description ="robot_description", bool prompt_before_exec =false)<br>constructor  |
| std::vector< moveit_msgs::CollisionObject > | **[get_collision_objects](Classes/classfranka__interface_1_1FrankaInterface.md#function-get-collision-objects)**()<br>Get all custom collision objects in the planning scene.  |
| void | **[grab_object](Classes/classfranka__interface_1_1FrankaInterface.md#function-grab-object)**(double width, double force)<br>Grasp an object with the gripper.  |
| void | **[lin_abs](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs)**(geometry_msgs::Pose pose, std::string frame_id ="panda_link0", std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot in a straight line to an absolute pose in Cartesian space.  |
| void | **[lin_abs](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs)**(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot in a straight line to an absolute pose in Cartesian space.  |
| void | **[lin_abs_subdivided](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs-subdivided)**(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name ="panda_hand_tcp")<br>Move the end effector of the robot in a straight line to an absolute pose in Cartesian space.  |
| void | **[lin_rel](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-rel)**(geometry_msgs::Pose rel_pose, std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot in a straight line to a pose relative to the current pose in Cartesian space.  |
| void | **[lin_rel_subdivided](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-rel-subdivided)**(geometry_msgs::Pose rel_pose, std::string end_effector_name ="panda_hand_tcp")<br>Move the end effector of the robot in a straight line to a pose relative to the current pose in Cartesian space.  |
| void | **[open_gripper](Classes/classfranka__interface_1_1FrankaInterface.md#function-open-gripper)**()<br>Opens the gripper completely.  |
| void | **[ptp_abs](Classes/classfranka__interface_1_1FrankaInterface.md#function-ptp-abs)**(geometry_msgs::Pose pose, std::string frame_id ="panda_link0", std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot to an absolute pose in Cartesian space.  |
| void | **[ptp_abs](Classes/classfranka__interface_1_1FrankaInterface.md#function-ptp-abs)**(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot to an absolute pose.  |
| void | **[ptp_abs](Classes/classfranka__interface_1_1FrankaInterface.md#function-ptp-abs)**(std::vector< double > goal_joints, std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot to an absolute pose in joint space.  |
| void | **[ptp_rel](Classes/classfranka__interface_1_1FrankaInterface.md#function-ptp-rel)**(geometry_msgs::Pose rel_pose, std::string end_effector_name ="panda_hand_tcp", bool prompt =false)<br>Move the end effector of the robot to a pose relative to the current pose in Cartesian space.  |
| void | **[remove_collision_object](Classes/classfranka__interface_1_1FrankaInterface.md#function-remove-collision-object)**(std::string id)<br>Remove a collision object from the planning scene.  |
| void | **[set_acceleration_scaling_factor](Classes/classfranka__interface_1_1FrankaInterface.md#function-set-acceleration-scaling-factor)**(double acceleration_scaling_factor)<br>change the maximum acceleration with that moveit will plan a ptp motion  |
| void | **[set_gripper_width](Classes/classfranka__interface_1_1FrankaInterface.md#function-set-gripper-width)**(double width)<br>Set the width of the gripper to a specific value.  |
| void | **[set_max_lin_velocity](Classes/classfranka__interface_1_1FrankaInterface.md#function-set-max-lin-velocity)**(double max_lin_velocity)<br>Set the maximum linear velocity for MoveIt to use when planning and executing a motion of the end effector.  |
| void | **[set_tolerances](Classes/classfranka__interface_1_1FrankaInterface.md#function-set-tolerances)**(double tolerance_pos, double tolerance_angle)<br>Set the tolerances for MoveIt to use when planning a motion.  |
| void | **[set_velocity_scaling_factor](Classes/classfranka__interface_1_1FrankaInterface.md#function-set-velocity-scaling-factor)**(double velocity_scaling_factor)<br>Set the velocity scaling factor for MoveIt to use when planning a point-to-point motion.  |
| void | **[visualize_point](Classes/classfranka__interface_1_1FrankaInterface.md#function-visualize-point)**(geometry_msgs::PoseStamped pose, std::string text ="goal")<br>Visualize a pose as a coordinate system in RViz.  |
| | **[~FrankaInterface](Classes/classfranka__interface_1_1FrankaInterface.md#function-~frankainterface)**() |

## Public Functions Documentation

### function activate_collision_check

```cpp
inline void activate_collision_check()
```

Activates the collision check for the planning scene. 

### function add_collision_object

```cpp
inline void add_collision_object(
    moveit_msgs::CollisionObject collision_object
)
```

Add a collision object to the planning scene. 

**Parameters**: 

  * **collision_object** The collision object to add.


**Exceptions**: 

  * **std::runtime_error** if the command fails. 


This function adds a collision object to the planning scene. The collision object is represented by a `moveit_msgs::CollisionObject` message, which contains the object's geometry and pose.


### function close_gripper

```cpp
void close_gripper()
```

Closes the gripper completely. 

**Exceptions**: 

  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 


This function sends a command to the gripper to close completely. The gripper will remain closed until a new command is sent to it. Note that this function is not suitable for grasping objects! Use the `grab_object` function instead.


### function deactivate_collision_check

```cpp
inline void deactivate_collision_check()
```

Deactivates the collision check for the planning scene. 

### function FrankaInterface

```cpp
FrankaInterface(
    ros::NodeHandle & nh,
    std::string robot_description ="robot_description",
    bool prompt_before_exec =false
)
```

constructor 

**Parameters**: 

  * **nh** node handle 


### function get_collision_objects

```cpp
std::vector< moveit_msgs::CollisionObject > get_collision_objects()
```

Get all custom collision objects in the planning scene. 

**Return**: A vector of collision objects. 

This function returns a vector of all custom collision objects in the planning scene. Custom collision objects are objects that have been added to the planning scene using the `add_collision_object` function.


### function grab_object

```cpp
void grab_object(
    double width,
    double force
)
```

Grasp an object with the gripper. 

**Parameters**: 

  * **width** The desired width of the gripper in meters. 
  * **force** The desired grasping force in Newtons.


**Exceptions**: 

  * **[ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** if the execution fails. 


This function sends a command to the gripper to grasp an object with the specified width and grasping force. The gripper will remain closed until a new command is sent to it.


### function lin_abs

```cpp
void lin_abs(
    geometry_msgs::Pose pose,
    std::string frame_id ="panda_link0",
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot in a straight line to an absolute pose in Cartesian space. 

**Parameters**: 

  * **pose** The absolute pose to move the end effector to. 
  * **frame_id** The frame ID of the pose. Default is "panda_link0". 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **std::runtime_error** if there was a communication error with ROS 
  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 
  * **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** if the planning failed to cover the whole path. In this case try subdividing the path with [lin_abs_subdivided()](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs-subdivided). 
  * **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** if the execution fails. 


This function generates a motion plan to move the end effector of the robot in a straight line to the specified absolute pose in Cartesian space.


### function lin_abs

```cpp
void lin_abs(
    geometry_msgs::PoseStamped goal_pose,
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot in a straight line to an absolute pose in Cartesian space. 

**Parameters**: 

  * **goal_pose** The absolute pose to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **std::runtime_error** if there was a communication error with ROS 
  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 
  * **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** if the planning failed to cover the whole path. In this case try subdividing the path with [lin_abs_subdivided()](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs-subdivided). 
  * **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** if the execution fails. 


This function generates a motion plan to move the end effector of the robot in a straight line to the specified absolute pose in Cartesian space.


### function lin_abs_subdivided

```cpp
void lin_abs_subdivided(
    geometry_msgs::PoseStamped goal_pose,
    std::string end_effector_name ="panda_hand_tcp"
)
```

Move the end effector of the robot in a straight line to an absolute pose in Cartesian space. 

**Parameters**: 

  * **goal_pose** The absolute pose to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **std::runtime_error** if there was a communication error with ROS 
  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 
  * **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** if the planning failed to cover the whole path. In this case try subdividing the path with [lin_abs_subdivided()](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs-subdivided). 
  * **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** if the execution fails. 


This function subdivides the path (multiple times if necessary) if the planning fails to cover the whole path.


### function lin_rel

```cpp
void lin_rel(
    geometry_msgs::Pose rel_pose,
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot in a straight line to a pose relative to the current pose in Cartesian space. 

**Parameters**: 

  * **rel_pose** The pose relative to the current pose to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **std::runtime_error** if there was a communication error with ROS 
  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 
  * **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** if the planning failed to cover the whole path. In this case try subdividing the path with [lin_abs_subdivided()](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs-subdivided). 
  * **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** if the execution fails. 


This function generates a motion plan to move the end effector of the robot in a straight line to the specified pose relative to the current pose in Cartesian space.


### function lin_rel_subdivided

```cpp
void lin_rel_subdivided(
    geometry_msgs::Pose rel_pose,
    std::string end_effector_name ="panda_hand_tcp"
)
```

Move the end effector of the robot in a straight line to a pose relative to the current pose in Cartesian space. 

**Parameters**: 

  * **rel_pose** The pose relative to the current pose to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **std::runtime_error** if there was a communication error with ROS 
  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 
  * **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** if the planning failed to cover the whole path. In this case try subdividing the path with [lin_abs_subdivided()](Classes/classfranka__interface_1_1FrankaInterface.md#function-lin-abs-subdivided). 
  * **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** if the execution fails. 


This function subdivides the path (multiple times if necessary) if the planning fails to cover the whole path.


### function open_gripper

```cpp
void open_gripper()
```

Opens the gripper completely. 

**Exceptions**: 

  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 


This function sends a command to the gripper to open completely. The gripper will remain open until a new command is sent to it.


### function ptp_abs

```cpp
void ptp_abs(
    geometry_msgs::Pose pose,
    std::string frame_id ="panda_link0",
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot to an absolute pose in Cartesian space. 

**Parameters**: 

  * **pose** The absolute pose to move the end effector to. 
  * **frame_id** The frame ID of the pose. Default is "panda_link0". 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 


This function generates a motion plan to move the end effector of the robot to the specified absolute pose in Cartesian space.


### function ptp_abs

```cpp
void ptp_abs(
    geometry_msgs::PoseStamped goal_pose,
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot to an absolute pose. 

**Parameters**: 

  * **goal_pose** The absolute pose to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 


This function generates a motion plan to move the end effector of the robot to the specified absolute pose.


### function ptp_abs

```cpp
void ptp_abs(
    std::vector< double > goal_joints,
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot to an absolute pose in joint space. 

**Parameters**: 

  * **goal_joints** The absolute joint positions to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **std::invalid_argument** if the number of joints in goal_joints is not equal to seven. 
  * **std::invalid_argument** if any of the joint positions is not within the joint limits. 
  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 
  * **std::runtime_error** if there was a communication error with ROS 


This function generates a motion plan to move the end effector of the robot to the specified absolute pose in joint space.


### function ptp_rel

```cpp
void ptp_rel(
    geometry_msgs::Pose rel_pose,
    std::string end_effector_name ="panda_hand_tcp",
    bool prompt =false
)
```

Move the end effector of the robot to a pose relative to the current pose in Cartesian space. 

**Parameters**: 

  * **rel_pose** The pose relative to the current pose to move the end effector to. 
  * **end_effector_name** The name of the end effector link. Default is "panda_hand_tcp". 
  * **prompt** If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.


**Exceptions**: 

  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 


This function generates a motion plan to move the end effector of the robot to the specified pose relative to the current pose in Cartesian space.


### function remove_collision_object

```cpp
inline void remove_collision_object(
    std::string id
)
```

Remove a collision object from the planning scene. 

**Parameters**: 

  * **id** The ID of the collision object to remove.


**Exceptions**: 

  * **std::runtime_error** if the command fails. 


This function removes a collision object from the planning scene using its ID.


### function set_acceleration_scaling_factor

```cpp
void set_acceleration_scaling_factor(
    double acceleration_scaling_factor
)
```

change the maximum acceleration with that moveit will plan a ptp motion 

**Parameters**: 

  * **acceleration_scaling_factor** percentage of the configured maximum acceleration (gets maximum value from parameter server) 


### function set_gripper_width

```cpp
void set_gripper_width(
    double width
)
```

Set the width of the gripper to a specific value. 

**Parameters**: 

  * **width** The desired width of the gripper in meters.


**Exceptions**: 

  * **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** if the planning fails. 


This function sends a command to the gripper to move to the specified width. Note that this function is not suitable for grasping objects! Use the `grab_object` function instead.


### function set_max_lin_velocity

```cpp
void set_max_lin_velocity(
    double max_lin_velocity
)
```

Set the maximum linear velocity for MoveIt to use when planning and executing a motion of the end effector. 

**Parameters**: 

  * **max_lin_velocity** The maximum linear velocity in m/s. 


This function sets the maximum linear velocity for MoveIt to use when planning and executing a motion of the end effector. The velocity curve of a linear motion has an acceleration and deceleration phase. The maximum velocity is only reached in the middle of the motion.


### function set_tolerances

```cpp
void set_tolerances(
    double tolerance_pos,
    double tolerance_angle
)
```

Set the tolerances for MoveIt to use when planning a motion. 

**Parameters**: 

  * **tolerance_pos** The tolerance for the position. 
  * **tolerance_angle** The tolerance for the orientation. 


This function sets the tolerances for MoveIt to use when planning a motion. The tolerances are used to determine how close the robot needs to get to the goal position and orientation before the motion is considered successful.


### function set_velocity_scaling_factor

```cpp
void set_velocity_scaling_factor(
    double velocity_scaling_factor
)
```

Set the velocity scaling factor for MoveIt to use when planning a point-to-point motion. 

**Parameters**: 

  * **velocity_scaling_factor** The percentage of the configured maximum velocity to use for planning the motion. 


This function sets the velocity scaling factor for MoveIt to use when planning a point-to-point motion. The velocity scaling factor is a percentage of the configured maximum velocity, which is obtained from the parameter server. A value of 1.0 means that MoveIt will plan the motion at the maximum velocity.


### function visualize_point

```cpp
void visualize_point(
    geometry_msgs::PoseStamped pose,
    std::string text ="goal"
)
```

Visualize a pose as a coordinate system in RViz. 

**Parameters**: 

  * **pose** The pose to visualize. 
  * **text** The text to display next to the coordinate system. Default is "goal". 


This function creates a coordinate system in RViz to visualize the specified pose. The coordinate system consists of three arrows representing the x, y, and z axes, and a label indicating the pose's position and orientation. The label can be customized with the optional `text` parameter.


### function ~FrankaInterface

```cpp
~FrankaInterface()
```


-------------------------------

Updated on 2023-07-10 at 09:42:18 +0200