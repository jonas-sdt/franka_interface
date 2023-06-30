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
    auto corner1 = make_pose_stamped(30_cm, 0_m, 5_cm, 180_deg, 0_deg, 0_deg, "world");
    auto corner2 = make_pose_stamped(65_cm, 0_m, 5_cm, 180_deg, 0_deg, 0_deg, "world");
    auto corner3 = make_pose_stamped(65_cm, 45_cm, 5_cm, 180_deg, 0_deg, 0_deg, "world");
    auto corner4 = make_pose_stamped(30_cm, 45_cm, 5_cm, 180_deg, 0_deg, 0_deg, "world");
    auto center  = make_pose_stamped(47.5_cm, 22.5_cm, 5_cm, 180_deg, 0_deg, 0_deg, "world");

    robot.ptp_abs(corner1);

    while(ros::ok())
    {
        // Put code that should run repeatedly here
        robot.lin_abs(corner2);
        robot.lin_abs(corner3);
        robot.lin_abs(center);
        robot.lin_abs(corner4);
        robot.lin_abs(corner1);
        ROS_INFO("Loop complete");
    }
    return 0;
}
