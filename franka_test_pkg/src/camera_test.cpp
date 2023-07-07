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

  // Put code that should run once here
  robot.ptp_abs(make_pose_stamped(40_cm, 0_cm, 35_cm, 180_deg, 20_deg, 180_deg, "world"));
  robot.lin_abs(make_pose_stamped(40_cm, 0_cm, 35_cm, 180_deg, 20_deg, 180_deg, "world"), "panda_hand_cam", true);
  robot.ptp_abs(make_pose_stamped(40_cm, 0_cm, 35_cm, 180_deg, 20_deg, 180_deg, "world"));
  robot.ptp_abs(make_pose_stamped(40_cm, 0_cm, 35_cm, 180_deg, 20_deg, 180_deg, "world"), "panda_hand_cam", true);

  return 0;
}
