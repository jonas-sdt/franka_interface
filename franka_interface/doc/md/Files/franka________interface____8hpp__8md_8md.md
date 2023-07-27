---
title: franka____interface__8hpp_8md.md

---

# franka____interface__8hpp_8md.md






## Source code

```markdown
---
title: franka__interface_8hpp.md

---

# franka__interface_8hpp.md






## Source code

```markdown
---
title: franka_interface/franka_interface.hpp

---

# franka_interface/franka_interface.hpp



## Namespaces

| Name           |
| -------------- |
| **[franka_interface](Namespaces/namespacefranka__interface.md)**  |

## Classes

|                | Name           |
| -------------- | -------------- |
| class | **[franka_interface::FrankaInterface](Classes/classfranka__interface_1_1FrankaInterface.md)**  |




## Source code

```cpp
#ifndef FRANKA_INTERFACE_H
#define FRANKA_INTERFACE_H

#include "actionlib/client/simple_action_client.h"
#include "franka_interface/exceptions.hpp"
#include "franka_gripper/GraspAction.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "moveit_msgs/GetCartesianPath.h"
#include "moveit_msgs/MoveGroupGoal.h"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "ros/ros.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cassert>

namespace franka_interface
{

  class FrankaInterface
  {

  public:
    FrankaInterface(ros::NodeHandle &nh, std::string robot_description = "robot_description", bool prompt_before_exec=false);

    ~FrankaInterface();

    void ptp_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void ptp_abs(std::vector<double> goal_joints, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void ptp_abs(geometry_msgs::Pose pose, std::string frame_id = "panda_link0", std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void ptp_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void lin_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void lin_abs_subdivided(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp");

    void lin_abs(geometry_msgs::Pose pose, std::string frame_id = "panda_link0", std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void lin_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    void lin_rel_subdivided(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp");

    void set_tolerances(double tolerance_pos, double tolerance_angle);

    void set_max_lin_velocity(double max_lin_velocity);

    void set_velocity_scaling_factor(double velocity_scaling_factor);

    void set_acceleration_scaling_factor(double acceleration_scaling_factor);

    void open_gripper();

    void close_gripper();

    void set_gripper_width(double width);

    void grab_object(double width, double force);

    void visualize_point(geometry_msgs::PoseStamped pose, std::string text = "goal");

    inline void add_collision_object(moveit_msgs::CollisionObject collision_object);

    inline void remove_collision_object(std::string id);

    std::vector<moveit_msgs::CollisionObject> get_collision_objects();

    inline void activate_collision_check();
    

    inline void deactivate_collision_check();
  
  private:

    inline geometry_msgs::PoseStamped ee_tf(const geometry_msgs::PoseStamped & pose, const std::string & end_effector_name);
    inline bool has_transform_changed(const std::string & source_frame, const std::string & target_frame);
    inline void init_planning_scene();

    ros::NodeHandle &nh_;
    double acceleration_scaling_factor_;
    bool activate_visualizations_;
    ros::ServiceClient cartesian_path_service_;
    std::vector<moveit_msgs::CollisionObject> custom_collision_objects_;
    std::vector<moveit_msgs::CollisionObject> default_collision_objects_;
    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> execute_trajectory_action_client_;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_action_client_;
    std::vector<std::pair<double, double>> joint_limits_;
    double max_lin_velocity_;
    moveit::planning_interface::MoveGroupInterfacePtr mgi_arm_;
    moveit::planning_interface::MoveGroupInterfacePtr mgi_gripper_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    bool prompt_before_exec_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    moveit::core::RobotModelPtr robot_model_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    double tolerance_angle_;
    double tolerance_pos_;
    double velocity_scaling_factor_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;
  };

} // namespace franka_interface

#endif
```


-------------------------------

Updated on 2023-07-10 at 09:42:18 +0200
```


-------------------------------

Updated on 2023-07-11 at 08:37:05 +0200
```


-------------------------------

Updated on 2023-07-27 at 16:29:38 +0200
