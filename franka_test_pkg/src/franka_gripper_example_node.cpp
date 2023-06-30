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
    

    while(ros::ok())
    {
        // Put code that should run repeatedly here
        robot.open_gripper();
        robot.close_gripper();
        robot.set_gripper_width(5_cm);
        ros::Duration(1.0).sleep();
    }
    
    return 0;
}
