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

  robot.open_gripper();
  robot.set_gripper_width(5_cm);
  robot.grab_object(2_cm, 4, true);
  ros::Duration(2).sleep();
  robot.open_gripper();

  return 0;
}
