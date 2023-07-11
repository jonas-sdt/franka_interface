---
title: namespacefranka__interface.md

---

# namespacefranka__interface.md






## Source code

```markdown
---
title: franka_interface

---

# franka_interface



## Classes

|                | Name           |
| -------------- | -------------- |
| class | **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** <br>Exception thrown when an execution operation fails.  |
| class | **[franka_interface::FrankaInterface](Classes/classfranka__interface_1_1FrankaInterface.md)**  |
| class | **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** <br>Exception thrown when a linear planning operation fails due to the goal pose not being reached.  |
| class | **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** <br>Exception thrown when a planning operation fails.  |

## Types

|                | Name           |
| -------------- | -------------- |
| typedef std::vector< double > | **[JointPositions](Namespaces/namespacefranka__interface.md#typedef-jointpositions)**  |

## Functions

|                | Name           |
| -------------- | -------------- |
| [JointPositions](Namespaces/namespacefranka__interface.md#typedef-jointpositions) | **[make_joint_state_goal](Namespaces/namespacefranka__interface.md#function-make-joint-state-goal)**(double q1, double q2, double q3, double q4, double q5, double q6, double q7)<br>create a joint state goal message  |
| [JointPositions](Namespaces/namespacefranka__interface.md#typedef-jointpositions) | **[make_joint_state_goal](Namespaces/namespacefranka__interface.md#function-make-joint-state-goal)**(long double q1, long double q2, long double q3, long double q4, long double q5, long double q6, long double q7)<br>create a joint state goal message  |
| geometry_msgs::Pose | **[make_pose](Namespaces/namespacefranka__interface.md#function-make-pose)**(double x, double y, double z, double qx, double qy, double qz, double qw)<br>create a pose message  |
| geometry_msgs::Pose | **[make_pose](Namespaces/namespacefranka__interface.md#function-make-pose)**(double x, double y, double z, double roll, double pitch, double yaw)<br>create a pose message  |
| geometry_msgs::PoseStamped | **[make_pose_stamped](Namespaces/namespacefranka__interface.md#function-make-pose-stamped)**(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id)<br>create a pose stamped message  |
| geometry_msgs::PoseStamped | **[make_pose_stamped](Namespaces/namespacefranka__interface.md#function-make-pose-stamped)**(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id)<br>create a pose stamped message  |
| long double | **[operator""_cm](Namespaces/namespacefranka__interface.md#function-operator""-cm)**(long double value) |
| long double | **[operator""_cm](Namespaces/namespacefranka__interface.md#function-operator""-cm)**(unsigned long long int value) |
| long double | **[operator""_deg](Namespaces/namespacefranka__interface.md#function-operator""-deg)**(long double value) |
| long double | **[operator""_deg](Namespaces/namespacefranka__interface.md#function-operator""-deg)**(unsigned long long int value) |
| long double | **[operator""_m](Namespaces/namespacefranka__interface.md#function-operator""-m)**(long double value) |
| long double | **[operator""_m](Namespaces/namespacefranka__interface.md#function-operator""-m)**(unsigned long long int value) |
| long double | **[operator""_mm](Namespaces/namespacefranka__interface.md#function-operator""-mm)**(long double value) |
| long double | **[operator""_mm](Namespaces/namespacefranka__interface.md#function-operator""-mm)**(unsigned long long int value) |
| long double | **[operator""_rad](Namespaces/namespacefranka__interface.md#function-operator""-rad)**(long double value) |
| long double | **[operator""_rad](Namespaces/namespacefranka__interface.md#function-operator""-rad)**(unsigned long long int value) |
| std::ostream & | **[operator<<](Namespaces/namespacefranka__interface.md#function-operator<<)**(std::ostream & os, const geometry_msgs::Pose & pose) |
| std::ostream & | **[operator<<](Namespaces/namespacefranka__interface.md#function-operator<<)**(std::ostream & os, const geometry_msgs::PoseStamped & pose_stamped) |
| std::string | **[pose_stamped_to_string](Namespaces/namespacefranka__interface.md#function-pose-stamped-to-string)**(const geometry_msgs::PoseStamped & pose_stamped)<br>create a string from a pose stamped message  |
| std::string | **[pose_to_string](Namespaces/namespacefranka__interface.md#function-pose-to-string)**(const geometry_msgs::Pose & pose)<br>create a string from a pose message  |

## Types Documentation

### typedef JointPositions

```cpp
typedef std::vector<double> franka_interface::JointPositions;
```



## Functions Documentation

### function make_joint_state_goal

```cpp
JointPositions make_joint_state_goal(
    double q1,
    double q2,
    double q3,
    double q4,
    double q5,
    double q6,
    double q7
)
```

create a joint state goal message 

**Parameters**: 

  * **q1** joint 1 position in radians 


### function make_joint_state_goal

```cpp
JointPositions make_joint_state_goal(
    long double q1,
    long double q2,
    long double q3,
    long double q4,
    long double q5,
    long double q6,
    long double q7
)
```

create a joint state goal message 

**Parameters**: 

  * **q1** joint 1 position in radians 


### function make_pose

```cpp
geometry_msgs::Pose make_pose(
    double x,
    double y,
    double z,
    double qx,
    double qy,
    double qz,
    double qw
)
```

create a pose message 

**Parameters**: 

  * **x** x position in meters 
  * **y** y position in meters 
  * **z** z position in meters 
  * **qx** x component of the quaternion 
  * **qy** y component of the quaternion 
  * **qz** z component of the quaternion 
  * **qw** w component of the quaternion 


### function make_pose

```cpp
geometry_msgs::Pose make_pose(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw
)
```

create a pose message 

**Parameters**: 

  * **x** x position in meters 
  * **y** y position in meters 
  * **z** z position in meters 
  * **roll** roll angle in radians 
  * **pitch** pitch angle in radians 
  * **yaw** yaw angle in radians 


### function make_pose_stamped

```cpp
geometry_msgs::PoseStamped make_pose_stamped(
    double x,
    double y,
    double z,
    double qx,
    double qy,
    double qz,
    double qw,
    std::string frame_id
)
```

create a pose stamped message 

**Parameters**: 

  * **x** x position in meters 
  * **y** y position in meters 
  * **z** z position in meters 
  * **qx** x component of the quaternion 
  * **qy** y component of the quaternion 
  * **qz** z component of the quaternion 
  * **qw** w component of the quaternion 
  * **frame_id** frame id of the pose 


### function make_pose_stamped

```cpp
geometry_msgs::PoseStamped make_pose_stamped(
    double x,
    double y,
    double z,
    double roll,
    double pitch,
    double yaw,
    std::string frame_id
)
```

create a pose stamped message 

**Parameters**: 

  * **x** x position in meters 
  * **y** y position in meters 
  * **z** z position in meters 
  * **roll** roll angle in radians 
  * **pitch** pitch angle in radians 
  * **yaw** yaw angle in radians 
  * **frame_id** frame id of the pose 


### function operator""_cm

```cpp
long double operator""_cm(
    long double value
)
```


### function operator""_cm

```cpp
long double operator""_cm(
    unsigned long long int value
)
```


### function operator""_deg

```cpp
long double operator""_deg(
    long double value
)
```


### function operator""_deg

```cpp
long double operator""_deg(
    unsigned long long int value
)
```


### function operator""_m

```cpp
long double operator""_m(
    long double value
)
```


### function operator""_m

```cpp
long double operator""_m(
    unsigned long long int value
)
```


### function operator""_mm

```cpp
long double operator""_mm(
    long double value
)
```


### function operator""_mm

```cpp
long double operator""_mm(
    unsigned long long int value
)
```


### function operator""_rad

```cpp
long double operator""_rad(
    long double value
)
```


### function operator""_rad

```cpp
long double operator""_rad(
    unsigned long long int value
)
```


### function operator<<

```cpp
std::ostream & operator<<(
    std::ostream & os,
    const geometry_msgs::Pose & pose
)
```


### function operator<<

```cpp
std::ostream & operator<<(
    std::ostream & os,
    const geometry_msgs::PoseStamped & pose_stamped
)
```


### function pose_stamped_to_string

```cpp
std::string pose_stamped_to_string(
    const geometry_msgs::PoseStamped & pose_stamped
)
```

create a string from a pose stamped message 

**Parameters**: 

  * **pose_stamped** pose stamped message 


### function pose_to_string

```cpp
std::string pose_to_string(
    const geometry_msgs::Pose & pose
)
```

create a string from a pose message 

**Parameters**: 

  * **pose** pose message 






-------------------------------

Updated on 2023-07-10 at 09:42:18 +0200
```


-------------------------------

Updated on 2023-07-11 at 08:37:05 +0200
