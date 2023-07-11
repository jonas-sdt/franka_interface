#include "ros/ros.h"
#include "franka_interface/franka_interface.hpp"
#include "franka_interface/utils.hpp"
#include "franka_interface/exceptions.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "template_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    using namespace franka_interface;
    FrankaInterface robot(nh);

    // Put code that should run once here
    auto start_pos = make_pose_stamped(30_cm, 0_m, 5_cm, 0_deg, 180_deg, 0_deg, "world");
    auto step_1 = make_pose(-35_cm,  0_cm, 0_cm, 0_deg, 0_deg, 0_deg);
    auto step_2 = make_pose(  0_cm, 45_cm, 0_cm, 0_deg, 0_deg, 0_deg);
    auto step_3 = make_pose( 35_cm,  0_cm, 0_cm, 0_deg, 0_deg, 0_deg);
    auto step_4 = make_pose(  0_cm,-45_cm, 0_cm, 0_deg, 0_deg, 0_deg);
    auto reduce_height = make_pose(0_cm, 0_cm, 5_mm, 0_deg, 0_deg, 0_deg);

    // auto corner1 = make_pose_stamped(30_cm, 0_m, 2_cm, 180_deg, 0_deg, 0_deg, "world");
    // auto corner2 = make_pose_stamped(65_cm, 0_m, 2_cm, 180_deg, 0_deg, 0_deg, "world");
    // auto corner3 = make_pose_stamped(65_cm, 45_cm, 2_cm, 180_deg, 0_deg, 0_deg, "world");
    // auto corner4 = make_pose_stamped(30_cm, 45_cm, 2_cm, 180_deg, 0_deg, 0_deg, "world");
    // auto center  = make_pose_stamped(47.5_cm, 22.5_cm, 2_cm, 180_deg, 0_deg, 0_deg, "world");

    robot.ptp_abs(start_pos);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    int i = 8;

    while(ros::ok() && i-->0)
    {
        // Put code that should run repeatedly here
        robot.lin_rel_subdivided(step_1);
        robot.lin_rel_subdivided(step_2);
        robot.lin_rel_subdivided(step_3);
        robot.lin_rel_subdivided(step_4);
        robot.lin_rel(reduce_height);
        ROS_INFO("Loop complete");
        ROS_INFO_STREAM("Current TCP Height: " << tf_buffer.lookupTransform("world", "panda_hand_tcp", ros::Time(0)).transform.translation.z);
    }
    return 0;
}
