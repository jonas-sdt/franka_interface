#ifndef FRANKA_INTERFACE_H
#define FRANKA_INTERFACE_H

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/MoveGroupGoal.h"
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include "sensor_msgs/JointState.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include <actionlib/client/simple_action_client.h>
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "franka_gripper/GraspAction.h"
#include "franka_gripper/MoveAction.h"
#include "franka_gripper/HomingAction.h"

#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <string>
#include <vector>
#include <memory>

namespace franka_interface
{

geometry_msgs::PoseStamped make_pose_stamped(float x, float y, float z, float qx, float qy, float qz, float qw, std::string frame_id);
geometry_msgs::Pose make_pose(float x, float y, float z, float qx, float qy, float qz, float qw);

class FrankaInterface
{

public:
    /**
     * \brief constructor
     * \param nh node handle
     */
    FrankaInterface(ros::NodeHandle &nh, std::string robot_description = "robot_description");

    ~FrankaInterface();

    /**
     * \brief ptp motion to an absolute pose
     * \param goal_pose absolute pose
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     */
    void ptp_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);
    
    /**
     * \brief ptp motion to an absolute pose in joint space
     * \todo fix this function
     * \param goal_joints absolute joint positions
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     * \throws std::invalid_argument if the number of joints in goal_joints is not equal to seven
     * \throws std::invalid_argument if any of the joint positions is not within the joint limits
     * \throws std::runtime_error if the planning fails
     */
    void ptp_abs(std::vector<double> goal_joints, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    /**
     * \brief ptp motion to an absolute pose
     * \param pose absolute pose
     * \param frame_id frame id of the pose
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     */
    void ptp_abs(geometry_msgs::Pose pose, std::string frame_id = "panda_link0", std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    /**
     * \brief relative ptp motion
     * \param rel_pose goal pose relative to the current pose
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     */
    void ptp_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    /**
     * \brief linear motion to an absolute pose
     * \param goal_pose absolute pose
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     */
    void lin_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    /**
     * \brief linear motion to an absolute pose
     * \param pose absolute pose
     * \param frame_id frame id of the pose
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     */
    void lin_abs(geometry_msgs::Pose pose, std::string frame_id = "panda_link0", std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    /**
     * \brief relative linear motion
     * \param rel_pose goal pose relative to the current pose
     * \param end_effector_name name of the end effector link
     * \param prompt if set to true, you'll have to press next in the rviz visualization tools gui to execute the plan
     */
    void lin_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

    /**
     * \brief change the tolerances with that moveit will plan the motion
     * \param tolerance_pose tolerance for the position
     * \param tolerance_angle tolerance for the orientation
     */
    void set_tolerances(double tolerance_pos, double tolerance_angle);

    /**
     * \brief change the maximum linear velocity in m/s with that moveit will plan and execute the motion of the end effector. The velocity curve of a linear motion has an acceleration and deceleration phase. The maximum velocity is only reached in the middle of the motion.
     */
    void set_max_lin_velocity(double max_lin_velocity);

    /**
     * \brief change the maximum velocity with that moveit will plan a ptp motion
     * \param velocity_scaling_factor percentage of the configured maximum velocity (gets maximum value from parameter server)
     */
    void set_velocity_scaling_factor(double velocity_scaling_factor);

    /**
     * \brief change the maximum acceleration with that moveit will plan a ptp motion
     * \param acceleration_scaling_factor percentage of the configured maximum acceleration (gets maximum value from parameter server)
     */
    void set_acceleration_scaling_factor(double acceleration_scaling_factor);

    /**
     * \brief opens the gripper completely
    */
    void open_gripper();

    /**
     * \brief closes the gripper completely. This function is not suitable for grasping objects! Use grab_object instead.
    */
    void close_gripper();

    /**
     * \brief move the gripper to a specific width. This function is not suitable for grasping objects! Use grab_object instead.
     * \param width width of the gripper in m
     */
    void set_gripper_width(double width);

    /**
     * \brief grab an object with the gripper
     * \param width width of the gripper in m
     * \param force grasping force in N
    */
    void grab_object(double width, double force);

    /**
     * \brief visualize a pose as a coordinate system
     * \param pose pose to visualize
     * \param text text to display next to the coordinate system
     * \param color color of text
     */
    void visualize_point(geometry_msgs::PoseStamped pose, std::string text = "goal");

    /**
     * \brief add a collision object to the planning scene
     * \param collision_object collision object to add
     */
    inline void add_collision_object(moveit_msgs::CollisionObject collision_object);

    /**
     * \brief remove a collision object from the planning scene
     * \param id collision object id
     */
    inline void remove_collision_object(std::string id);

    /**
     * \brief get all custom collision objects in the planning scene
     * \return vector of collision objects
     */
    std::vector<moveit_msgs::CollisionObject> get_collision_objects();


private:
    void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
    inline void init_planning_scene();
    inline void activate_table_collision_check();
    inline void deactivate_table_collision_check();

    inline void send_planning_request(planning_interface::MotionPlanRequest &request, planning_interface::MotionPlanResponse &response);

    ros::NodeHandle &nh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    bool activate_visualizations_;
    bool prompt_before_exec_;
    moveit_visual_tools::MoveItVisualTools visual_tools_;

    moveit::core::RobotModelPtr robot_model_;
    planning_pipeline::PlanningPipelinePtr planning_pipeline_;
    ros::Subscriber joint_state_subscriber_;
    // actionlib::SimpleActionClient<moveit_msgs::MoveGroupGoal> move_action_client_;
    ros::ServiceClient cartesian_path_service_;
    actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> execute_trajectory_action_client_;
    actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_action_client_;
    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_move_action_client_;
    actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_homing_action_client_;
    franka_gripper::MoveGoal gripper_open_goal_;
    franka_gripper::MoveGoal gripper_close_goal_;

    double tolerance_pos_;
    double tolerance_angle_;
    double velocity_scaling_factor_;
    double acceleration_scaling_factor_;
    double max_lin_velocity_;
    std::vector<std::pair<double, double>> joint_limits_;
    moveit::planning_interface::MoveGroupInterfacePtr mgi_arm_;
    moveit::planning_interface::MoveGroupInterfacePtr mgi_gripper_;

    sensor_msgs::JointState current_joint_state_;
    planning_scene_monitor::PlanningSceneMonitorPtr psm_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::vector<moveit_msgs::CollisionObject> default_collision_objects_;
    std::vector<moveit_msgs::CollisionObject> custom_collision_objects_;
};

} // namespace franka_interface

#endif