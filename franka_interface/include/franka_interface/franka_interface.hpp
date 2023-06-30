#ifndef FRANKA_INTERFACE_H
#define FRANKA_INTERFACE_H

#include "actionlib/client/simple_action_client.h"
#include "franka_interface/exceptions.hpp"
#include "franka_gripper/GraspAction.h"
#include "franka_gripper/HomingAction.h"
#include "franka_gripper/MoveAction.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit_msgs/CollisionObject.h"
#include "moveit_msgs/ExecuteTrajectoryAction.h"
#include "moveit_msgs/GetCartesianPath.h"
#include "moveit_msgs/MoveGroupGoal.h"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "ros/ros.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include "sensor_msgs/JointState.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/utils.h"
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <cassert>

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
         * \brief Move the end effector of the robot to an absolute pose.
         *
         * This function generates a motion plan to move the end effector of the robot to the specified absolute pose.
         *
         * \param goal_pose The absolute pose to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         * 
         * \throws franka_interface::PlanningFailed if the planning fails.
         */
        void ptp_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

        /**
         * \brief Move the end effector of the robot to an absolute pose in joint space.
         *
         * This function generates a motion plan to move the end effector of the robot to the specified absolute pose in joint space.
         *
         * \param goal_joints The absolute joint positions to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         *
         * \throws std::invalid_argument if the number of joints in goal_joints is not equal to seven.
         * \throws std::invalid_argument if any of the joint positions is not within the joint limits.
         * \throws franka_interface::PlanningFailed if the planning fails.
         * \throws std::runtime_error if there was a communication error with ROS
         */
        void ptp_abs(std::vector<double> goal_joints, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

        /**
         * \brief Move the end effector of the robot to an absolute pose in Cartesian space.
         *
         * This function generates a motion plan to move the end effector of the robot to the specified absolute pose in Cartesian space.
         *
         * \param pose The absolute pose to move the end effector to.
         * \param frame_id The frame ID of the pose. Default is "panda_link0".
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         *
         * \throws franka_interface::PlanningFailed if the planning fails.
         */
        void ptp_abs(geometry_msgs::Pose pose, std::string frame_id = "panda_link0", std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

        /**
         * \brief Move the end effector of the robot to a pose relative to the current pose in Cartesian space.
         *
         * This function generates a motion plan to move the end effector of the robot to the specified pose relative to the current pose in Cartesian space.
         *
         * \param rel_pose The pose relative to the current pose to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         *
         * \throws franka_interface::PlanningFailed if the planning fails.
         */
        void ptp_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

        /**
         * \brief Move the end effector of the robot in a straight line to an absolute pose in Cartesian space.
         *
         * This function generates a motion plan to move the end effector of the robot in a straight line to the specified absolute pose in Cartesian space.
         *
         * \param goal_pose The absolute pose to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         *
         * \throws std::runtime_error if there was a communication error with ROS
         * \throws franka_interface::PlanningFailed if the planning fails.
         * \throws franka_interface::LinPlanningFailedIncomplete if the planning failed to cover the whole path. In this case try subdividing the path with lin_abs_subdivided().
         * \throws franka_interface::ExecutionFailed if the execution fails.
         */
        void lin_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

        /**
         * \brief Move the end effector of the robot in a straight line to an absolute pose in Cartesian space. 
         * 
         * This function subdivides the path (multiple times if necessary) if the planning fails to cover the whole path.
         * 
         * \param goal_pose The absolute pose to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         * 
         * \throws std::runtime_error if there was a communication error with ROS
         * \throws franka_interface::PlanningFailed if the planning fails.
         * \throws franka_interface::LinPlanningFailedIncomplete if the planning failed to cover the whole path. In this case try subdividing the path with lin_abs_subdivided().
         * \throws franka_interface::ExecutionFailed if the execution fails.
        */
        void lin_abs_subdivided(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name = "panda_hand_tcp");

        /**
         * \brief Move the end effector of the robot in a straight line to an absolute pose in Cartesian space.
         *
         * This function generates a motion plan to move the end effector of the robot in a straight line to the specified absolute pose in Cartesian space.
         *
         * \param pose The absolute pose to move the end effector to.
         * \param frame_id The frame ID of the pose. Default is "panda_link0".
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         *
         * \throws std::runtime_error if there was a communication error with ROS
         * \throws franka_interface::PlanningFailed if the planning fails.
         * \throws franka_interface::LinPlanningFailedIncomplete if the planning failed to cover the whole path. In this case try subdividing the path with lin_abs_subdivided().
         * \throws franka_interface::ExecutionFailed if the execution fails.
         */
        void lin_abs(geometry_msgs::Pose pose, std::string frame_id = "panda_link0", std::string end_effector_name = "panda_hand_tcp", bool prompt = false);

        /**
         * \brief Move the end effector of the robot in a straight line to a pose relative to the current pose in Cartesian space.
         *
         * This function generates a motion plan to move the end effector of the robot in a straight line to the specified pose relative to the current pose in Cartesian space.
         *
         * \param rel_pose The pose relative to the current pose to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         *
         * \throws std::runtime_error if there was a communication error with ROS
         * \throws franka_interface::PlanningFailed if the planning fails.
         * \throws franka_interface::LinPlanningFailedIncomplete if the planning failed to cover the whole path. In this case try subdividing the path with lin_abs_subdivided().
         * \throws franka_interface::ExecutionFailed if the execution fails.
         */
        void lin_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp", bool prompt = false);


        /**
         * \brief Move the end effector of the robot in a straight line to a pose relative to the current pose in Cartesian space.
         * 
         * This function subdivides the path (multiple times if necessary) if the planning fails to cover the whole path.
         * 
         * \param rel_pose The pose relative to the current pose to move the end effector to.
         * \param end_effector_name The name of the end effector link. Default is "panda_hand_tcp".
         * \param prompt If set to true, the user will have to press "Next" in the RViz visualization tools GUI to execute the plan.
         * 
         * \throws std::runtime_error if there was a communication error with ROS
         * \throws franka_interface::PlanningFailed if the planning fails.
         * \throws franka_interface::LinPlanningFailedIncomplete if the planning failed to cover the whole path. In this case try subdividing the path with lin_abs_subdivided().
         * \throws franka_interface::ExecutionFailed if the execution fails.
        */
        void lin_rel_subdivided(geometry_msgs::Pose rel_pose, std::string end_effector_name = "panda_hand_tcp");

        /**
         * \brief Set the tolerances for MoveIt to use when planning a motion.
         *
         * This function sets the tolerances for MoveIt to use when planning a motion. The tolerances are used to determine how close the robot needs to get to the goal position and orientation before the motion is considered successful.
         *
         * \param tolerance_pos The tolerance for the position.
         * \param tolerance_angle The tolerance for the orientation.
         */
        void set_tolerances(double tolerance_pos, double tolerance_angle);

        /**
         * \brief Set the maximum linear velocity for MoveIt to use when planning and executing a motion of the end effector.
         *
         * This function sets the maximum linear velocity for MoveIt to use when planning and executing a motion of the end effector. The velocity curve of a linear motion has an acceleration and deceleration phase. The maximum velocity is only reached in the middle of the motion.
         *
         * \param max_lin_velocity The maximum linear velocity in m/s.
         */
        void set_max_lin_velocity(double max_lin_velocity);

        /**
         * \brief Set the velocity scaling factor for MoveIt to use when planning a point-to-point motion.
         *
         * This function sets the velocity scaling factor for MoveIt to use when planning a point-to-point motion. The velocity scaling factor is a percentage of the configured maximum velocity, which is obtained from the parameter server. A value of 1.0 means that MoveIt will plan the motion at the maximum velocity.
         *
         * \param velocity_scaling_factor The percentage of the configured maximum velocity to use for planning the motion.
         */
        void set_velocity_scaling_factor(double velocity_scaling_factor);

        /**
         * \brief change the maximum acceleration with that moveit will plan a ptp motion
         * \param acceleration_scaling_factor percentage of the configured maximum acceleration (gets maximum value from parameter server)
         */
        void set_acceleration_scaling_factor(double acceleration_scaling_factor);

        /**
         * \brief Opens the gripper completely.
         *
         * This function sends a command to the gripper to open completely. The gripper will remain open until a new command is sent to it.
         *
         * \throws franka_interface::PlanningFailed if the planning fails.
         */
        void open_gripper();

        /**
         * \brief Closes the gripper completely.
         *
         * This function sends a command to the gripper to close completely. The gripper will remain closed until a new command is sent to it. Note that this function is not suitable for grasping objects! Use the `grab_object` function instead.
         *
         * \throws franka_interface::PlanningFailed if the planning fails.
         */
        void close_gripper();

        /**
         * \brief Set the width of the gripper to a specific value.
         *
         * This function sends a command to the gripper to move to the specified width. Note that this function is not suitable for grasping objects! Use the `grab_object` function instead.
         *
         * \param width The desired width of the gripper in meters.
         *
         * \throws franka_interface::PlanningFailed if the planning fails.
         */
        void set_gripper_width(double width);

        /**
         * \brief Grasp an object with the gripper.
         *
         * This function sends a command to the gripper to grasp an object with the specified width and grasping force. The gripper will remain closed until a new command is sent to it.
         *
         * \param width The desired width of the gripper in meters.
         * \param force The desired grasping force in Newtons.
         *
         * \throws ExecutionFailed if the execution fails.
         */
        void grab_object(double width, double force);

        /**
         * \brief Visualize a pose as a coordinate system in RViz.
         *
         * This function creates a coordinate system in RViz to visualize the specified pose. The coordinate system consists of three arrows representing the x, y, and z axes, and a label indicating the pose's position and orientation. The label can be customized with the optional `text` parameter.
         *
         * \param pose The pose to visualize.
         * \param text The text to display next to the coordinate system. Default is "goal".
         */
        void visualize_point(geometry_msgs::PoseStamped pose, std::string text = "goal");

        /**
         * \brief Add a collision object to the planning scene.
         *
         * This function adds a collision object to the planning scene. The collision object is represented by a `moveit_msgs::CollisionObject` message, which contains the object's geometry and pose.
         *
         * \param collision_object The collision object to add.
         *
         * \throws std::runtime_error if the command fails.
         */
        inline void add_collision_object(moveit_msgs::CollisionObject collision_object);

        /**
         * \brief Remove a collision object from the planning scene.
         *
         * This function removes a collision object from the planning scene using its ID.
         *
         * \param id The ID of the collision object to remove.
         *
         * \throws std::runtime_error if the command fails.
         */
        inline void remove_collision_object(std::string id);

        /**
         * \brief Get all custom collision objects in the planning scene.
         *
         * This function returns a vector of all custom collision objects in the planning scene. Custom collision objects are objects that have been added to the planning scene using the `add_collision_object` function.
         *
         * \return A vector of collision objects.
         */
        std::vector<moveit_msgs::CollisionObject> get_collision_objects();

    private:
        void joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg);
        inline void init_planning_scene();
        inline void activate_table_collision_check();
        inline void deactivate_table_collision_check();

        inline void send_planning_request(planning_interface::MotionPlanRequest &request, planning_interface::MotionPlanResponse &response);

        ros::NodeHandle &nh_;
        
        double acceleration_scaling_factor_;
        bool activate_visualizations_;
        ros::ServiceClient cartesian_path_service_;
        sensor_msgs::JointState current_joint_state_;
        std::vector<moveit_msgs::CollisionObject> custom_collision_objects_;
        std::vector<moveit_msgs::CollisionObject> default_collision_objects_;
        actionlib::SimpleActionClient<moveit_msgs::ExecuteTrajectoryAction> execute_trajectory_action_client_;
        actionlib::SimpleActionClient<franka_gripper::GraspAction> gripper_action_client_;
        franka_gripper::MoveGoal gripper_close_goal_;
        actionlib::SimpleActionClient<franka_gripper::HomingAction> gripper_homing_action_client_;
        actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_move_action_client_;
        franka_gripper::MoveGoal gripper_open_goal_;
        std::vector<std::pair<double, double>> joint_limits_;
        ros::Subscriber joint_state_subscriber_;
        double max_lin_velocity_;
        moveit::planning_interface::MoveGroupInterfacePtr mgi_arm_;
        moveit::planning_interface::MoveGroupInterfacePtr mgi_gripper_;
        planning_pipeline::PlanningPipelinePtr planning_pipeline_;
        moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
        bool prompt_before_exec_;
        planning_scene_monitor::PlanningSceneMonitorPtr psm_;
        moveit::core::RobotModelPtr robot_model_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
        double tolerance_angle_;
        double tolerance_pos_;
        double velocity_scaling_factor_;
        moveit_visual_tools::MoveItVisualTools visual_tools_;
    };

} // namespace franka_interface

#endif