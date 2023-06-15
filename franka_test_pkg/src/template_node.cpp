#include "ros/ros.h"
#include "franka_interface/franka_interface.hpp"
#include "franka_interface/utils.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    franka_interface::FrankaInterface robot(nh);

    // Put code that should run once here
    

    while(ros::ok())
    {
        // Put code that should run repeatedly here
        

    }
    return 0;
}
