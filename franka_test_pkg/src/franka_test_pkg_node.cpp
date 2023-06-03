#include "franka_interface/franka_interface.hpp"
#include <ros/ros.h>
#include "geometry_msgs/PoseStamped.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "franka_test_node");
  ros::NodeHandle nh;

  ROS_INFO("Starting franka_test_node");
  FrankaInterface franka_interface(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // franka_interface.open_gripper();
  // franka_interface.close_gripper();


  geometry_msgs::Pose pose_right;
  pose_right.position.x = 0;
  pose_right.position.y = -0.1;
  pose_right.position.z = 0;
  pose_right.orientation.w = 1.0;
  pose_right.orientation.x = 0.0;
  pose_right.orientation.y = 0.0;
  pose_right.orientation.z = 0.0;

  geometry_msgs::Pose pose_left;
  pose_left.position.x = 0;
  pose_left.position.y = 0.1;
  pose_left.position.z = 0;
  pose_left.orientation.w = 1.0;
  pose_left.orientation.x = 0.0;
  pose_left.orientation.y = 0.0;
  pose_left.orientation.z = 0.0;

  geometry_msgs::Pose pose_up;
  pose_up.position.x = 0;
  pose_up.position.y = 0;
  pose_up.position.z = -0.1;
  pose_up.orientation.w = 1.0;
  pose_up.orientation.x = 0.0;
  pose_up.orientation.y = 0.0;
  pose_up.orientation.z = 0.0;

  geometry_msgs::Pose pose_down;
  pose_down.position.x = 0;
  pose_down.position.y = 0;
  pose_down.position.z = 0.1;
  pose_down.orientation.w = 1.0;
  pose_down.orientation.x = 0.0;
  pose_down.orientation.y = 0.0;
  pose_down.orientation.z = 0.0;

  franka_interface.set_max_lin_velocity(0.1);

  franka_interface.lin_rel(pose_left, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_left, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_up, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_up, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_right, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_right, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_down, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.lin_rel(pose_down, "panda_hand_tcp");
  ros::Duration(0.25).sleep();

  franka_interface.ptp_rel(pose_left, "panda_hand_tcp");


  while (ros::ok())
  {
    ROS_INFO_ONCE("franka_test_node in idle loop");
  }

  return 0;
}