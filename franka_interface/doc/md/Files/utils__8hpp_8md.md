---
title: utils_8hpp.md

---

# utils_8hpp.md






## Source code

```markdown
---
title: franka_interface/utils.hpp

---

# franka_interface/utils.hpp



## Namespaces

| Name           |
| -------------- |
| **[franka_interface](Namespaces/namespacefranka__interface.md)**  |




## Source code

```cpp
#ifndef FRANKA_INTERFACE_UTILS_HPP
#define FRANKA_INTERFACE_UTILS_HPP

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include <iostream>
#include <vector>
#include "moveit_msgs/CollisionObject.h"
#include "shape_msgs/SolidPrimitive.h"

namespace franka_interface
{

  typedef std::vector<double> JointPositions;

  moveit_msgs::CollisionObject make_collision_object(std::string id, geometry_msgs::PoseStamped pose, shape_msgs::SolidPrimitive primitive);

  JointPositions make_joint_state_goal(long double q1, long double q2, long double q3, long double q4, long double q5, long double q6, long double q7);

  JointPositions make_joint_state_goal(double q1, double q2, double q3, double q4, double q5, double q6, double q7);

  geometry_msgs::PoseStamped make_pose_stamped(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id);

  geometry_msgs::PoseStamped make_pose_stamped(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id);

  geometry_msgs::Pose make_pose(double x, double y, double z, double qx, double qy, double qz, double qw);

  geometry_msgs::Pose make_pose(double x, double y, double z, double roll, double pitch, double yaw);

  std::string pose_stamped_to_string(const geometry_msgs::PoseStamped &pose_stamped);

  std::string pose_to_string(const geometry_msgs::Pose &pose);

  std::ostream &operator<<(std::ostream &os, const geometry_msgs::PoseStamped &pose_stamped);

  std::ostream &operator<<(std::ostream &os, const geometry_msgs::Pose &pose);

  long double operator"" _m(unsigned long long int value);

  long double operator"" _m(long double value);

  long double operator"" _cm(unsigned long long int value);

  long double operator"" _cm(long double value);

  long double operator"" _mm(unsigned long long int value);

  long double operator"" _mm(long double value);

  long double operator"" _deg(unsigned long long int value);

  long double operator"" _deg(long double value);

  long double operator"" _rad(unsigned long long int value);

  long double operator"" _rad(long double value);

} // namespace franka_interface

#endif // FRANKA_INTERFACE_UTILS_HPP
```


-------------------------------

Updated on 2023-07-11 at 08:37:05 +0200
```


-------------------------------

Updated on 2023-07-27 at 16:29:38 +0200
