---
title: utils__8cpp_8md.md

---

# utils__8cpp_8md.md






## Source code

```markdown
---
title: utils_8cpp.md

---

# utils_8cpp.md






## Source code

```markdown
---
title: franka_interface/utils.cpp

---

# franka_interface/utils.cpp



## Namespaces

| Name           |
| -------------- |
| **[franka_interface](Namespaces/namespacefranka__interface.md)**  |




## Source code

```cpp
#include "franka_interface/utils.hpp"

namespace franka_interface
{

  JointPositions make_joint_state_goal(long double q1, long double q2, long double q3, long double q4, long double q5, long double q6, long double q7)
  {
    JointPositions joint_positions;
    joint_positions.push_back((double)q1);
    joint_positions.push_back((double)q2);
    joint_positions.push_back((double)q3);
    joint_positions.push_back((double)q4);
    joint_positions.push_back((double)q5);
    joint_positions.push_back((double)q6);
    joint_positions.push_back((double)q7);
    return joint_positions;
  }

  JointPositions make_joint_state_goal(double q1, double q2, double q3, double q4, double q5, double q6, double q7)
  {
    JointPositions joint_positions;
    joint_positions.push_back(q1);
    joint_positions.push_back(q2);
    joint_positions.push_back(q3);
    joint_positions.push_back(q4);
    joint_positions.push_back(q5);
    joint_positions.push_back(q6);
    joint_positions.push_back(q7);
    return joint_positions;
  }

  geometry_msgs::PoseStamped make_pose_stamped(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id)
  {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    pose_stamped.pose.orientation.x = qx;
    pose_stamped.pose.orientation.y = qy;
    pose_stamped.pose.orientation.z = qz;
    pose_stamped.pose.orientation.w = qw;
    pose_stamped.header.frame_id = frame_id;
    return pose_stamped;
  }

  geometry_msgs::PoseStamped make_pose_stamped(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return make_pose_stamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame_id);
  }

  geometry_msgs::Pose make_pose(double x, double y, double z, double qx, double qy, double qz, double qw)
  {
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.x = qx;
    pose.orientation.y = qy;
    pose.orientation.z = qz;
    pose.orientation.w = qw;
    return pose;
  }

  geometry_msgs::Pose make_pose(double x, double y, double z, double roll, double pitch, double yaw)
  {
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return make_pose(x, y, z, q.x(), q.y(), q.z(), q.w());
  }

  std::string pose_stamped_to_string(const geometry_msgs::PoseStamped &pose_stamped)
  {
    std::string str = "PoseStamped: ";
    str += "frame_id: " + pose_stamped.header.frame_id + ", \n";
    str += "position: (" + std::to_string(pose_stamped.pose.position.x) + ", " + std::to_string(pose_stamped.pose.position.y) + ", " + std::to_string(pose_stamped.pose.position.z) + "), \n";
    str += "orientation: (" + std::to_string(pose_stamped.pose.orientation.x) + ", " + std::to_string(pose_stamped.pose.orientation.y) + ", " + std::to_string(pose_stamped.pose.orientation.z) + ", " + std::to_string(pose_stamped.pose.orientation.w) + ")";
    return str;
  }

  std::string pose_to_string(const geometry_msgs::Pose &pose)
  {
    std::string str = "Pose: ";
    str += "position: (" + std::to_string(pose.position.x) + ", " + std::to_string(pose.position.y) + ", " + std::to_string(pose.position.z) + "), \n";
    str += "orientation: (" + std::to_string(pose.orientation.x) + ", " + std::to_string(pose.orientation.y) + ", " + std::to_string(pose.orientation.z) + ", " + std::to_string(pose.orientation.w) + ")";
    return str;
  }

  std::ostream &operator<<(std::ostream &os, const geometry_msgs::PoseStamped &pose_stamped)
  {
    os << pose_stamped_to_string(pose_stamped);
    return os;
  }

  std::ostream &operator<<(std::ostream &os, const geometry_msgs::Pose &pose)
  {
    os << pose_to_string(pose);
    return os;
  }

  long double operator"" _m(unsigned long long int value)
  {
    return value;
  }

  long double operator"" _m(long double value)
  {
    return value;
  }

  long double operator"" _cm(unsigned long long int value)
  {
    return value / 100.0;
  }

  long double operator"" _cm(long double value)
  {
    return value / 100.0;
  }

  long double operator"" _mm(unsigned long long int value)
  {
    return value / 1000.0;
  }

  long double operator"" _mm(long double value)
  {
    return value / 1000.0;
  }

  long double operator"" _deg(unsigned long long int value)
  {
    return value * M_PI / 180.0;
  }

  long double operator"" _deg(long double value)
  {
    return value * M_PI / 180.0;
  }

  long double operator"" _rad(unsigned long long int value)
  {
    return value;
  }

  long double operator"" _rad(long double value)
  {
    return value;
  }
} // namespace franka_interface
```


-------------------------------

Updated on 2023-07-10 at 09:26:48 +0200
```


-------------------------------

Updated on 2023-07-10 at 09:42:18 +0200
```


-------------------------------

Updated on 2023-07-11 at 08:37:04 +0200
