#include <ros/ros.h>
#include "franka_interface/franka_interface.hpp"
#include "franka_interface/utils.hpp"
#include "geometry_msgs/PoseStamped.h"
#include <random>

int main(int argc, char** argv)
{
  using namespace franka_interface;
  ros::init(argc, argv, "franka_test_node");
  ros::NodeHandle nh;

  ROS_INFO("Starting franka_test_node");
  FrankaInterface franka_interface(nh);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  // franka_interface.open_gripper();
  // franka_interface.close_gripper();

  auto pose_home  = make_pose(0.307_m, -0.000_m, 0.487_m, 180_deg, 0_deg, 0_deg);
  auto pose_right = make_pose(0_m, -0.1_m,   0_m, 0_deg, 0_deg, 20_deg);
  auto pose_left  = make_pose(0_m,  0.1_m,   0_m, 0_deg, 0_deg,-20_deg);
  auto pose_up    = make_pose(0_m,  0_m,   0.1_m, 0_deg, 0_deg, 20_deg);
  auto pose_down  = make_pose(0_m,  0_m,  -0.1_m, 0_deg, 0_deg,-20_deg);

  franka_interface.set_velocity_scaling_factor(0.5);
  franka_interface.set_acceleration_scaling_factor(0.5);
  franka_interface.ptp_abs(pose_home);

  ROS_INFO_STREAM("Moving to pose_right: " << pose_right);
  franka_interface.set_max_lin_velocity(0.2);
  franka_interface.lin_rel(pose_left, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_max_lin_velocity(0.05);
  franka_interface.lin_rel(pose_left, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_max_lin_velocity(0.02);
  franka_interface.lin_rel(pose_right, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_max_lin_velocity(0.05);
  franka_interface.lin_rel(pose_right, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_velocity_scaling_factor(0.2);
  franka_interface.ptp_rel(pose_left, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_velocity_scaling_factor(0.05);
  franka_interface.ptp_rel(pose_left, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_velocity_scaling_factor(0.2);
  franka_interface.ptp_rel(pose_right, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  franka_interface.set_velocity_scaling_factor(0.05);
  franka_interface.ptp_rel(pose_right, "panda_hand_tcp", true);
  ros::Duration(2).sleep();

  while (ros::ok())
  {
    ROS_INFO_ONCE("franka_test_node in idle loop");
  }

  return 0;
}