#include "ros/ros.h"
#include "franka_interface/franka_interface.hpp"
#include "franka_interface/utils.hpp"
#include "franka_interface/exceptions.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "template_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();

  using namespace franka_interface;
  FrankaInterface robot(nh);

  // Put code that should run once here

  // create box primitive for box
  shape_msgs::SolidPrimitive box_primitive;
  box_primitive.type = box_primitive.BOX;
  box_primitive.dimensions.resize(3);
  box_primitive.dimensions[0] = 0.1;
  box_primitive.dimensions[1] = 0.1;
  box_primitive.dimensions[2] = 0.1;
  geometry_msgs::PoseStamped box_pose = make_pose_stamped(0.4,0.0,0.3, 20_deg, 20_deg, 20_deg, "world");
  robot.add_collision_object(make_collision_object("box", box_pose, box_primitive));   
 
  while(ros::ok())
  {
      // Put code that should run repeatedly here
      robot.ptp_abs(make_pose_stamped(0.25,0.0,0.3, 180_deg, 0_deg, 0_deg, "world"));
      ros::Duration(5).sleep();
      robot.deactivate_collision_check();

      robot.ptp_abs(make_pose_stamped(0.55,0.0,0.3, 180_deg, 0_deg, 0_deg, "world"));
      ros::Duration(5).sleep();
      robot.activate_collision_check();
  }
  return 0;
}
