#include "franka_interface/franka_interface.hpp"
#include "moveit/kinematic_constraints/utils.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit_msgs/GetCartesianPath.h"
#include <vector>
#include <algorithm>

FrankaInterface::FrankaInterface(ros::NodeHandle &nh, std::string robot_description)
    : nh_(nh),
      tf_listener_(tf_buffer_),
      activate_visualizations_(true),
      prompt_before_exec_(false),
      visual_tools_("panda_link0"),
      joint_state_subscriber_(nh_.subscribe("/joint_states", 1, &FrankaInterface::joint_state_callback, this)),
      tolerance_pos_(0.001),
      tolerance_angle_(0.01),
      max_lin_velocity_(0.1),
      velocity_scaling_factor_(0.1),
      acceleration_scaling_factor_(0.1),
      planning_scene_interface_(),
      execute_trajectory_action_client_("execute_trajectory", true),
      gripper_action_client_("franka_gripper/grasp", true),
      gripper_move_action_client_("franka_gripper/move", true),
      gripper_homing_action_client_("franka_gripper/homing", true),
      spinner_(1)
{
    // initialize variables that depend on the robot model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_model_loader);
    planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_loader->getModel(), nh, "planning_plugin", "request_adapters"),

    // initialize planning scene
        init_planning_scene();

    // initialize action clients
    cartesian_path_service_ =
        nh_.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path");

    // gripper positions
    gripper_open_goal_.width = 0.08;
    gripper_open_goal_.speed = 0.1;

    gripper_close_goal_.width = 0.0;
    gripper_close_goal_.speed = 0.1;

    joint_limits_ = {
        std::make_pair(-2.897246558, 2.897246558), 
        std::make_pair(-1.832595715, 1.832595715),
        std::make_pair(-2.897246558, 2.897246558),
        std::make_pair(-3.071779484, -0.122173048),
        std::make_pair(-2.879793266, 2.879793266),
        std::make_pair(0.436332313, 4.625122518),
        std::make_pair(-3.054326191, 3.054326191)
    };                                                      // joint limits saved as a pair of min and max values

    spinner_.start();
    mgi_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_arm");
}

FrankaInterface::~FrankaInterface()
{
    spinner_.stop();
    visual_tools_.deleteAllMarkers();
    deactivate_table_collision_check();
}

void FrankaInterface::send_planning_request(planning_interface::MotionPlanRequest &req, planning_interface::MotionPlanResponse &res)
{
    // get current planning scene from planning scene monitor
    if (!psm_->requestPlanningSceneState("/get_planning_scene"))
    {
        ROS_ERROR_STREAM("Could not get planning scene");
        throw std::runtime_error("Could not get planning scene");
    }

    if (planning_pipeline_ && psm_)
    {
        planning_scene_monitor::LockedPlanningSceneRO planning_scene(psm_);

        // compute plan
        planning_pipeline_->generatePlan(planning_scene, req, res);

        // check if planning was successful
        if (res.error_code_.val != res.error_code_.SUCCESS)
        {
            ROS_ERROR_STREAM("Could not compute PTP plan successfully. Error Code: " + std::to_string(res.error_code.val));
            throw std::runtime_error("Could not compute PTP plan successfully");
        }
    }
    else
    {
        // handle null pointers
        ROS_ERROR("Planning pipeline or planning scene monitor is null");
        throw std::runtime_error("Planning pipeline or planning scene monitor is null");
    }
}

void FrankaInterface::ptp_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name, bool prompt)
{

    if (activate_visualizations_)
    {
        visual_tools_.deleteAllMarkers();
        visualize_point(goal_pose, "PTP Goal");
    }

    const moveit::core::JointModelGroup* joint_model_group = mgi_->getCurrentState()->getJointModelGroup("panda_arm");

    // transform goal pose to panda_link0 frame
    geometry_msgs::PoseStamped goal_pose_transformed;
    goal_pose_transformed.header.frame_id = goal_pose.header.frame_id;
    goal_pose_transformed.pose = goal_pose.pose;
    tf_buffer_.transform(goal_pose_transformed, goal_pose_transformed, "panda_link0");
    
    mgi_->setEndEffectorLink(end_effector_name);
    mgi_->setPlanningTime(5.0);
    mgi_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    mgi_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    mgi_->setPlannerId("PTP");
    mgi_->setPoseTarget(goal_pose_transformed, end_effector_name);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (mgi_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group);
        visual_tools_.trigger();
        mgi_->execute(plan);
    }
    else
    {
        ROS_ERROR_STREAM("Could not compute PTP plan successfully");
        throw std::runtime_error("Could not compute PTP plan successfully");
    }
}

void FrankaInterface::ptp_abs(std::vector<double> joint_space_goal, std::string end_effector_name, bool prompt)
{
    if (joint_space_goal.size() != 7)
    {
        ROS_ERROR_STREAM("Goal joint vector has wrong size. Expected 7, got " + std::to_string(joint_space_goal.size()));
        throw std::invalid_argument("Goal joint vector has wrong size");
    }
    
    // check if goal is within joint limits
    for(int i = 0; i < 7; i++)
    {
        if(joint_space_goal[i] != std::clamp(joint_space_goal[i], joint_limits_[i].first, joint_limits_[i].second))
        {
            ROS_ERROR_STREAM("Goal position for joint " + std::to_string(i) + " is out of joint limits");
            throw std::invalid_argument("Goal position for joint " + std::to_string(i) + " is out of joint limits");
        }
    }

    // get current robot state
    moveit::core::RobotState goal_state = planning_scene_monitor::LockedPlanningSceneRO(psm_)->getCurrentState();
    
    // set joint values
    goal_state.setJointGroupPositions(goal_state.getRobotModel()->getJointModelGroup("panda_arm"), joint_space_goal);

    // create joint goal
    moveit_msgs::Constraints joint_goal = kinematic_constraints::constructGoalConstraints(goal_state, goal_state.getRobotModel()->getJointModelGroup("panda_arm"));
    
    // create a motion plan request
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    req.group_name = "panda_arm";
    req.planner_id = "PTP";
    req.pipeline_id = "pilz_industrial_motion_planner";
    req.num_planning_attempts = 5;
    req.allowed_planning_time = 5.0;
    req.max_velocity_scaling_factor = velocity_scaling_factor_;
    req.max_acceleration_scaling_factor = acceleration_scaling_factor_;
    req.goal_constraints.push_back(joint_goal);

    // send planning request
    send_planning_request(req, res);
}

void FrankaInterface::ptp_abs(geometry_msgs::Pose pose, std::string frame_id, std::string end_effector_name, bool prompt)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = pose;
    goal_pose.header.frame_id = frame_id;
    ptp_abs(goal_pose, end_effector_name);
}

void FrankaInterface::ptp_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name, bool prompt)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = rel_pose;
    goal_pose.header.frame_id = "panda_hand_tcp";
    ptp_abs(goal_pose, end_effector_name);
}

void FrankaInterface::lin_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name, bool prompt)
{
    if (activate_visualizations_)
    {
        visual_tools_.deleteAllMarkers();
        visualize_point(goal_pose, "Lin Goal");
    }

    // create trajectory
    moveit_msgs::RobotTrajectory trajectory;
    moveit_msgs::GetCartesianPathRequest cartesian_path_req;
    moveit_msgs::GetCartesianPathResponse cartesian_path_res;

    cartesian_path_req.group_name = "panda_manipulator";
    cartesian_path_req.header.frame_id = goal_pose.header.frame_id;
    cartesian_path_req.header.stamp = ros::Time::now();
    cartesian_path_req.waypoints.push_back(goal_pose.pose);
    cartesian_path_req.max_step = 0.01;
    cartesian_path_req.jump_threshold = 0.0;
    cartesian_path_req.avoid_collisions = true;
    cartesian_path_req.cartesian_speed_limited_link = end_effector_name;
    cartesian_path_req.max_cartesian_speed = max_lin_velocity_;

    if (cartesian_path_service_.call(cartesian_path_req, cartesian_path_res))
    {
        if (cartesian_path_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            ROS_ERROR("Could not compute cartesian path");
            throw std::runtime_error("Could not compute cartesian path");
        }
    }
    else
    {
        ROS_ERROR("Service call failed");
        throw std::runtime_error("Service call failed");
    }

    trajectory = cartesian_path_res.solution;

    if (prompt_before_exec_ || prompt)
    {
        visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to start execution of the lin motion");
    }

    // execute trajectory
    moveit_msgs::ExecuteTrajectoryGoal execute_trajectory_goal;
    execute_trajectory_goal.trajectory = trajectory;

    execute_trajectory_action_client_.sendGoal(execute_trajectory_goal);

    // wait for the action to return
    bool finished_before_timeout = execute_trajectory_action_client_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = execute_trajectory_action_client_.getState();
        ROS_INFO("Trajectory execution action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Trajectory execution action did not finish before the time out.");
        execute_trajectory_action_client_.cancelGoal();
        throw std::runtime_error("Trajectory execution action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (execute_trajectory_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR_STREAM("Could not execute trajectory successfully");
        return;
    }

    // reset action client
    execute_trajectory_action_client_.cancelGoal();
}

void FrankaInterface::lin_abs(geometry_msgs::Pose pose, std::string frame_id, std::string end_effector_name, bool prompt)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = pose;
    goal_pose.header.frame_id = frame_id;
    lin_abs(goal_pose, end_effector_name);
}

void FrankaInterface::lin_rel(geometry_msgs::Pose rel_pose, std::string end_effector_name, bool prompt)
{
    geometry_msgs::PoseStamped goal_pose;
    goal_pose.pose = rel_pose;
    goal_pose.header.frame_id = end_effector_name;
    lin_abs(goal_pose, end_effector_name);
}

void FrankaInterface::set_tolerances(double tolerance_pos, double tolerance_angle)
{
    // TODO: check if tolerances are valid

    tolerance_pos_ = tolerance_pos;
    tolerance_angle_ = tolerance_angle;
}

void FrankaInterface::set_max_lin_velocity(double max_lin_velocity)
{
    // TODO: check if max_lin_velocity is valid

    max_lin_velocity_ = max_lin_velocity;
}

void FrankaInterface::set_velocity_scaling_factor(double velocity_scaling_factor)
{
    // TODO: check if velocity_scaling_factor is valid

    velocity_scaling_factor_ = velocity_scaling_factor;
}

void FrankaInterface::set_acceleration_scaling_factor(double acceleration_scaling_factor)
{
    // TODO: check if acceleration_scaling_factor is valid

    acceleration_scaling_factor_ = acceleration_scaling_factor;
}

void FrankaInterface::open_gripper()
{
    gripper_move_action_client_.sendGoal(gripper_open_goal_);

    // wait for the action to return
    // wait for the action to return
    bool finished_before_timeout = gripper_move_action_client_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = gripper_move_action_client_.getState();
        ROS_INFO("Open Gripper action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Open Gripper action did not finish before the time out.");
        gripper_move_action_client_.cancelGoal();
        throw std::runtime_error("Open Gripper action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (gripper_move_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR_STREAM("Could not open gripper successfully");
        throw std::runtime_error("Could not open gripper successfully");
    }
}

void FrankaInterface::close_gripper()
{
    gripper_move_action_client_.sendGoal(gripper_close_goal_);

    // wait for the action to return
    // wait for the action to return
    bool finished_before_timeout = gripper_move_action_client_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = gripper_move_action_client_.getState();
        ROS_INFO("Close Gripper action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Close Gripper action did not finish before the time out.");
        gripper_move_action_client_.cancelGoal();
        throw std::runtime_error("Close Gripper action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (gripper_move_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR_STREAM("Could not close gripper successfully");
        throw std::runtime_error("Could not close gripper successfully");
    }
}

void FrankaInterface::set_gripper_width(double width)
{
    // TODO: check if width is within limits

    franka_gripper::MoveGoal goal;
    goal.width = width;
    goal.speed = 0.1;

    gripper_move_action_client_.sendGoal(goal);

    // wait for the action to return
    bool finished_before_timeout = gripper_move_action_client_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = gripper_move_action_client_.getState();
        ROS_INFO("Set gripper width action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Set gripper width action did not finish before the time out.");
        gripper_move_action_client_.cancelGoal();
        throw std::runtime_error("Set gripper width action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (gripper_move_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR_STREAM("Could not set gripper width successfully");
        throw std::runtime_error("Could not set gripper width successfully");
    }
}

void FrankaInterface::grab_object(double width, double force)
{
    // TODO: check if width is within limits
    // TODO: check if force is within limits

    franka_gripper::GraspGoal goal;
    goal.width = width;
    goal.force = 4;
    goal.speed = 1.0;
    goal.epsilon.inner = 0.01;
    goal.epsilon.outer = 0.01;

    gripper_action_client_.sendGoal(goal);

    // wait for the action to return
    // wait for the action to return
    bool finished_before_timeout = gripper_action_client_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = gripper_action_client_.getState();
        ROS_INFO("Grab object action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Grab object action did not finish before the time out.");
        gripper_action_client_.cancelGoal();
        throw std::runtime_error("Grab object action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (gripper_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_ERROR_STREAM("Could not grab object successfully");
        throw std::runtime_error("Could not grab object successfully");
    }
}

void FrankaInterface::visualize_point(geometry_msgs::PoseStamped pose, std::string text)
{
    pose = tf_buffer_.transform(pose, "panda_link0", ros::Duration(1.0));
    visual_tools_.publishAxisLabeled(pose.pose, text);
    visual_tools_.trigger();
}

inline void FrankaInterface::add_collision_object(moveit_msgs::CollisionObject collision_object)
{
    custom_collision_objects_.push_back(collision_object);
    planning_scene_interface_.addCollisionObjects(std::vector<moveit_msgs::CollisionObject>({collision_object}));
}

inline void FrankaInterface::remove_collision_object(std::string id)
{
    // TODO: check if object with id exists, otherwise throw error

    custom_collision_objects_.erase(std::remove_if(custom_collision_objects_.begin(), custom_collision_objects_.end(), [id](moveit_msgs::CollisionObject collision_object)
                                                   { return collision_object.id == id; }),
                                    custom_collision_objects_.end());
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>({id}));
}

std::vector<moveit_msgs::CollisionObject> FrankaInterface::get_collision_objects()
{
    return custom_collision_objects_;
}

inline void FrankaInterface::init_planning_scene()
{
    /* listen for planning scene messages on topic /XXX and apply them to
                 the internal planning scene accordingly */
    psm_->startSceneMonitor("/move_group/monitored_planning_scene");
    /* listens to changes of world geometry, collision objects, and (optionally) octomaps */
    psm_->startWorldGeometryMonitor();
    /* listen to joint state updates as well as changes in attached collision objects
                  and update the internal planning scene accordingly*/
    psm_->startStateMonitor();
    // // moveit::core::RobotStatePtr robot_state(
    // //     new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));
    // //
    // // const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup("panda_arm");
    // create box primitive for table
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 1;
    table_primitive.dimensions[1] = 1;
    table_primitive.dimensions[2] = 0.5;
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = 0.3;
    table_pose.position.y = 0.4;
    table_pose.position.z = -0.25;

    // create collision object for table
    moveit_msgs::CollisionObject table_collision_object;
    table_collision_object.header.frame_id = "world";
    table_collision_object.id = "table";
    table_collision_object.primitives.push_back(table_primitive);
    table_collision_object.primitive_poses.push_back(table_pose);
    table_collision_object.operation = table_collision_object.ADD;

    // create box primitive for camera stand
    shape_msgs::SolidPrimitive camera_stand_primitive;
    camera_stand_primitive.type = camera_stand_primitive.BOX;
    camera_stand_primitive.dimensions.resize(3);
    camera_stand_primitive.dimensions[0] = 0.1;
    camera_stand_primitive.dimensions[1] = 0.1;
    camera_stand_primitive.dimensions[2] = 1;

    geometry_msgs::Pose camera_stand_pose;
    camera_stand_pose.orientation.w = 1.0;
    camera_stand_pose.position.x = -0.1;
    camera_stand_pose.position.y = 0.7;
    camera_stand_pose.position.z = 0.5;

    // create collision object for camera stand
    moveit_msgs::CollisionObject camera_stand_collision_object;
    camera_stand_collision_object.header.frame_id = "world";
    camera_stand_collision_object.id = "camera_stand";
    camera_stand_collision_object.primitives.push_back(camera_stand_primitive);
    camera_stand_collision_object.primitive_poses.push_back(camera_stand_pose);
    camera_stand_collision_object.operation = camera_stand_collision_object.ADD;

    // create box primitive for monitor
    shape_msgs::SolidPrimitive monitor_primitive;
    monitor_primitive.type = monitor_primitive.BOX;
    monitor_primitive.dimensions.resize(3);
    monitor_primitive.dimensions[0] = 0.3;
    monitor_primitive.dimensions[1] = 0.45;
    monitor_primitive.dimensions[2] = 0.45;

    tf2::Quaternion q;
    q.setRPY(0.01, -0.01, -0.49);
    geometry_msgs::Pose monitor_pose;
    monitor_pose.orientation.w = q.w();
    monitor_pose.orientation.x = q.x();
    monitor_pose.orientation.y = q.y();
    monitor_pose.orientation.z = q.z();
    monitor_pose.position.x = 0.06;
    monitor_pose.position.y = 0.78;
    monitor_pose.position.z = 0.33;

    // create collision object for table
    moveit_msgs::CollisionObject monitor_collision_object;
    monitor_collision_object.header.frame_id = "world";
    monitor_collision_object.id = "monitor";
    monitor_collision_object.primitives.push_back(monitor_primitive);
    monitor_collision_object.primitive_poses.push_back(monitor_pose);
    monitor_collision_object.operation = monitor_collision_object.ADD;

    // add collision objects to collision_objects_ and planning scene
    default_collision_objects_.push_back(table_collision_object);
    default_collision_objects_.push_back(camera_stand_collision_object);
    default_collision_objects_.push_back(monitor_collision_object);
    planning_scene_interface_.applyCollisionObjects(default_collision_objects_);
}

inline void FrankaInterface::activate_table_collision_check()
{
    planning_scene_interface_.addCollisionObjects(default_collision_objects_);
    planning_scene_interface_.addCollisionObjects(custom_collision_objects_);
}

inline void FrankaInterface::deactivate_table_collision_check()
{
    // remove default collision objects
    planning_scene_interface_.removeCollisionObjects(std::vector<std::string>({"table", "camera_stand", "monitor"}));

    // put all custom collision object ids into vector
    std::vector<std::string> custom_collision_object_ids;
    std::transform(custom_collision_objects_.begin(), custom_collision_objects_.end(), std::back_inserter(custom_collision_object_ids),
                   [](auto &&obj)
                   { return obj.id; });

    // remove custom collision objects
    planning_scene_interface_.removeCollisionObjects(custom_collision_object_ids);
}

void FrankaInterface::joint_state_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    current_joint_state_ = *msg;
}
