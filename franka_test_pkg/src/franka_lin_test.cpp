#include "ros/ros.h"
#include "franka_interface/franka_interface.hpp"
#include "franka_interface/utils.hpp"
#include "franka_interface/exceptions.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "template_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  using namespace franka_interface;
  FrankaInterface robot(nh);

  robot.ptp_abs(make_pose_stamped(64_cm, 0_cm, 4_cm, 180_deg, 0_deg, 0_deg, "world"));

  while (ros::ok())
  {
    robot.set_max_lin_velocity(0.1);
    robot.lin_abs(make_pose_stamped(64_cm, 45_cm, 4_cm, 180_deg, 0_deg, 0_deg, "world"), "panda_hand_tcp", true);

    robot.set_max_lin_velocity(0.5);
    robot.lin_abs(make_pose_stamped(64_cm, 0_cm, 4_cm, 180_deg, 0_deg, 0_deg, "world"));
  }
  return 0;
}
