---
title: franka__interface_8cpp.md

---

# franka__interface_8cpp.md






## Source code

```markdown
---
title: franka_interface/franka_interface.cpp

---

# franka_interface/franka_interface.cpp



## Namespaces

| Name           |
| -------------- |
| **[franka_interface](Namespaces/namespacefranka__interface.md)**  |




## Source code

```cpp
#include "franka_interface/franka_interface.hpp"

namespace franka_interface
{

  FrankaInterface::FrankaInterface(ros::NodeHandle &nh, std::string robot_description, bool prompt_before_exec)
      : nh_(nh),
        acceleration_scaling_factor_(0.1),
        activate_visualizations_(true),
        cartesian_path_service_(nh_.serviceClient<moveit_msgs::GetCartesianPath>("/compute_cartesian_path")),
        execute_trajectory_action_client_("execute_trajectory", true),
        gripper_action_client_("franka_gripper/grasp", true),
        max_lin_velocity_(0.1),
        planning_scene_interface_(),
        prompt_before_exec_(prompt_before_exec),
        tf_listener_(tf_buffer_),
        tolerance_angle_(0.01),
        tolerance_pos_(0.001),
        velocity_scaling_factor_(0.1),
        visual_tools_("panda_link0")
  {
    // initialize variables that depend on the robot model
    robot_model_loader::RobotModelLoaderPtr robot_model_loader(new robot_model_loader::RobotModelLoader("robot_description"));
    psm_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(robot_model_loader);
    planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_loader->getModel(), nh, "planning_plugin", "request_adapters"),

    // initialize planning scene
    init_planning_scene();

    // joint limits saved as a pair of min and max values
    joint_limits_ = {
      std::make_pair(-2.897246558,  2.897246558),
      std::make_pair(-1.832595715,  1.832595715),
      std::make_pair(-2.897246558,  2.897246558),
      std::make_pair(-3.071779484, -0.122173048),
      std::make_pair(-2.879793266,  2.879793266),
      std::make_pair( 0.436332313,  4.625122518),
      std::make_pair(-3.054326191,  3.054326191)
    };

    // initialize move group interfaces
    mgi_arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_arm");
    mgi_arm_->setPlanningTime(5.0);
    mgi_arm_->setPlanningPipelineId("ompl");
    mgi_arm_->setPlannerId("PTP");
    mgi_arm_->setNumPlanningAttempts(10);
    mgi_arm_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    mgi_arm_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);

    mgi_gripper_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>("panda_hand");
    mgi_gripper_->setPlanningTime(3);
    mgi_gripper_->setPlanningPipelineId("ompl");
    mgi_gripper_->setPlannerId("PTP");
    mgi_gripper_->setMaxVelocityScalingFactor(0.1);
    mgi_gripper_->setMaxAccelerationScalingFactor(0.1);
  }

  FrankaInterface::~FrankaInterface()
  {
    visual_tools_.deleteAllMarkers();
    deactivate_collision_check();
  }

  void FrankaInterface::ptp_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name, bool prompt)
  {

    const moveit::core::JointModelGroup *joint_model_group = mgi_arm_->getCurrentState()->getJointModelGroup("panda_arm");

    // transform goal pose to panda_link0 frame
    geometry_msgs::PoseStamped goal_pose_transformed;
    goal_pose_transformed.header.frame_id = goal_pose.header.frame_id;
    goal_pose_transformed.pose = goal_pose.pose;
    tf_buffer_.transform(goal_pose_transformed, goal_pose_transformed, "panda_link0");

    if (activate_visualizations_)
    {
      visual_tools_.deleteAllMarkers();
      visualize_point(goal_pose, "PTP Goal");
      visual_tools_.trigger();
    }

    if (end_effector_name != "panda_hand_tcp")
    {
      if (has_transform_changed("panda_hand_tcp", end_effector_name))
      {
        ROS_ERROR_STREAM("The transform between panda_link0 and " << end_effector_name << " has changed. "
                                                                  << "This might lead to unexpected behavior.");
        throw std::runtime_error("Transform between panda_link0 and end effector has changed");
      }

      // compensate for transform between panda_hand_tcp and end_effector_name
      goal_pose_transformed = ee_tf(goal_pose_transformed, end_effector_name);
    }

    mgi_arm_->setEndEffectorLink("panda_hand_tcp");
    mgi_arm_->setMaxVelocityScalingFactor(velocity_scaling_factor_);
    mgi_arm_->setMaxAccelerationScalingFactor(acceleration_scaling_factor_);
    mgi_arm_->setPoseTarget(goal_pose_transformed);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error_code = mgi_arm_->plan(plan);
    if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Could not compute PTP plan successfully");
      throw PlanningFailed(error_code);
    }

    if (activate_visualizations_)
    {
      visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group);
      visual_tools_.trigger();
    }

    if (prompt_before_exec_ || prompt)
    {
      visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to start execution of the ptp motion");
    }

    mgi_arm_->execute(plan);
  }

  void FrankaInterface::ptp_abs(std::vector<double> joint_space_goal, std::string end_effector_name, bool prompt)
  {
    moveit::core::RobotStatePtr current_state = mgi_arm_->getCurrentState();
    const moveit::core::JointModelGroup *joint_model_group =
        mgi_arm_->getCurrentState()->getJointModelGroup("panda_arm");

    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

    // joint_group_positions[0] = -tau / 6;  // -1/6 turn in radians

    mgi_arm_->setJointValueTarget(joint_space_goal);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode error_code = mgi_arm_->plan(plan);

    if (error_code != moveit::core::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR_STREAM("Could not compute PTP plan successfully");
      throw PlanningFailed(error_code);
    }

    if (activate_visualizations_)
    {
      visual_tools_.deleteAllMarkers();
      visual_tools_.publishTrajectoryLine(plan.trajectory_, joint_model_group);
      visual_tools_.trigger();
    }

    if (prompt_before_exec_ || prompt)
    {
      visual_tools_.prompt("Press 'next' in the RvizVisualToolsGui window to start execution of the ptp motion");
    }

    mgi_arm_->execute(plan);
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
    ptp_abs(goal_pose, end_effector_name, prompt);
  }

  void FrankaInterface::lin_abs(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name, bool prompt)
  {
    if (activate_visualizations_)
    {
      visual_tools_.deleteAllMarkers();
      visualize_point(goal_pose, "Lin Goal");
    }

    if (end_effector_name != "panda_hand_tcp")
    {
      if (has_transform_changed("panda_hand_tcp", end_effector_name))
      {
        ROS_ERROR_STREAM("The transform between panda_link0 and " << end_effector_name << " has changed. "
                                                                  << "This might lead to unexpected behavior.");
        throw std::runtime_error("Transform between panda_link0 and end effector has changed");
      }

      // compensate for transform between panda_hand_tcp and end_effector_name
      goal_pose = ee_tf(goal_pose, end_effector_name);
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
    cartesian_path_req.jump_threshold = 5.0;
    cartesian_path_req.avoid_collisions = true;
    cartesian_path_req.cartesian_speed_limited_link = end_effector_name;
    cartesian_path_req.max_cartesian_speed = max_lin_velocity_;

    if (cartesian_path_service_.call(cartesian_path_req, cartesian_path_res))
    {
      if (cartesian_path_res.error_code.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
      {
        ROS_ERROR("Could not compute cartesian path");
        throw PlanningFailed(cartesian_path_res.error_code);
      }
    }
    else
    {
      ROS_ERROR("Service call failed");
      throw std::runtime_error("Service call failed");
    }

    trajectory = cartesian_path_res.solution;

    // check if the goal can be reached by this plan
    if (std::abs(cartesian_path_res.fraction - 1) > 0.0001)
    {
      ROS_ERROR_STREAM("Can only plan " << cartesian_path_res.fraction * 100 << " \% of LIN path. Aborted. Goal Pose was: \n"
                                        << goal_pose);
      throw LinPlanningFailedIncomplete(goal_pose, cartesian_path_res.fraction);
    }

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
      throw ExecutionFailed("Trajectory execution action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (execute_trajectory_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR_STREAM("Could not execute trajectory successfully");
      throw ExecutionFailed("Could not execute trajectory successfully");
      return;
    }

    // reset action client
    execute_trajectory_action_client_.cancelGoal();
  }

  void FrankaInterface::lin_abs_subdivided(geometry_msgs::PoseStamped goal_pose, std::string end_effector_name)
  {
    int lin_devider = 1;
    int todo_movements = 1;
    while (todo_movements > 0)
    {
      try
      {
        lin_abs(goal_pose, "panda_hand_tcp", true);
        ros::Duration(1).sleep();
        todo_movements--;
      }
      catch (const LinPlanningFailedIncomplete &e)
      {
        goal_pose.pose.position.x = goal_pose.pose.position.x / 2;
        goal_pose.pose.position.y = goal_pose.pose.position.y / 2;
        goal_pose.pose.position.z = goal_pose.pose.position.z / 2;
        // halve the rotation as well
        tf2::Quaternion q;
        tf2::fromMsg(goal_pose.pose.orientation, q);
        q = q.slerp(tf2::Quaternion::getIdentity(), 0.5);
        goal_pose.pose.orientation = tf2::toMsg(q);

        lin_devider++;
        todo_movements *= 2;
      }
      if (lin_devider > 10)
      {
        ROS_ERROR_STREAM("Could not complete LIN motion. Aborted. Goal Pose was: \n"
                         << goal_pose);
        throw ExecutionFailed("Could not complete LIN motion.");
      }
    }
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
    lin_abs(goal_pose, end_effector_name, prompt);
  }

  void FrankaInterface::lin_rel_subdivided(geometry_msgs::Pose rel_pose, std::string end_effector_name)
  {
    int lin_devider = 1;
    int todo_movements = 1;
    while (todo_movements > 0)
    {
      try
      {
        lin_rel(rel_pose, "panda_hand_tcp", true);
        ros::Duration(1).sleep();
        todo_movements--;
      }
      catch (const LinPlanningFailedIncomplete &e)
      {
        rel_pose.position.x = rel_pose.position.x / 2;
        rel_pose.position.y = rel_pose.position.y / 2;
        rel_pose.position.z = rel_pose.position.z / 2;

        // halve the rotation as well
        tf2::Quaternion q;
        tf2::fromMsg(rel_pose.orientation, q);
        q = q.slerp(tf2::Quaternion::getIdentity(), 0.5);
        rel_pose.orientation = tf2::toMsg(q);

        lin_devider++;
        todo_movements *= 2;
      }
      if (lin_devider > 10)
      {
        ROS_ERROR_STREAM("Could not complete LIN motion. Aborted. Goal Pose was: \n"
                         << rel_pose);
        throw ExecutionFailed("Could not complete LIN motion.");
      }
    }
  }

  void FrankaInterface::set_tolerances(double tolerance_pos, double tolerance_angle)
  {
    // TODO: check if tolerances are valid

    tolerance_pos_ = tolerance_pos;
    tolerance_angle_ = tolerance_angle;
  }

  void FrankaInterface::set_max_lin_velocity(double max_lin_velocity)
  {
    if (max_lin_velocity < 0.01 || max_lin_velocity > 2)
      throw std::invalid_argument("max_lin_velocity must be between 0.01 and 2");
    max_lin_velocity_ = max_lin_velocity;
  }

  void FrankaInterface::set_velocity_scaling_factor(double velocity_scaling_factor)
  {
    if (velocity_scaling_factor < 0.01 || velocity_scaling_factor > 1)
      throw std::invalid_argument("velocity_scaling_factor must be between 0.01 and 1");
    velocity_scaling_factor_ = velocity_scaling_factor;
  }

  void FrankaInterface::set_acceleration_scaling_factor(double acceleration_scaling_factor)
  {
    // TODO: check if acceleration_scaling_factor is valid

    acceleration_scaling_factor_ = acceleration_scaling_factor;
  }

  void FrankaInterface::open_gripper()
  {
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    mgi_gripper_->setJointValueTarget(mgi_gripper_->getNamedTargetValues("open"));
    moveit::core::MoveItErrorCode error_code = mgi_gripper_->plan(gripper_plan);

    if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Could not plan gripper motion");
      throw PlanningFailed(error_code);
    }

    mgi_gripper_->move();
  }

  void FrankaInterface::close_gripper()
  {
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    mgi_gripper_->setJointValueTarget(mgi_gripper_->getNamedTargetValues("close"));
    moveit::core::MoveItErrorCode error_code = mgi_gripper_->plan(gripper_plan);

    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Could not plan gripper motion");
      throw PlanningFailed(error_code);
    }

    mgi_gripper_->move();
  }

  void FrankaInterface::set_gripper_width(double width)
  {
    std::vector<double> finger_width;
    finger_width.resize(2);
    finger_width[0] = width;
    finger_width[1] = width;
    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    mgi_gripper_->setJointValueTarget(finger_width);
    moveit::core::MoveItErrorCode error_code = mgi_gripper_->plan(gripper_plan);

    if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
    {
      ROS_ERROR("Could not plan gripper motion");
      throw PlanningFailed(error_code);
    }

    mgi_gripper_->move();
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
      throw ExecutionFailed("Grab object action did not finish before the time out.");
    }

    // check if plan execution was successful
    if (gripper_action_client_.getState() != actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_ERROR_STREAM("Could not grab object successfully");
      throw ExecutionFailed("Could not grab object successfully");
    }
  }

  void FrankaInterface::visualize_point(geometry_msgs::PoseStamped pose, std::string text)
  {
    pose = tf_buffer_.transform(pose, "panda_link0", ros::Duration(1.0));
    visual_tools_.publishAxisLabeled(pose.pose, text);
    visual_tools_.trigger();
  }

  void FrankaInterface::add_collision_object(moveit_msgs::CollisionObject collision_object)
  {
    custom_collision_objects_.push_back(collision_object);
    planning_scene_interface_.addCollisionObjects(std::vector<moveit_msgs::CollisionObject>({collision_object}));
  }

  void FrankaInterface::remove_collision_object(std::string id)
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

  inline geometry_msgs::PoseStamped FrankaInterface::ee_tf(const geometry_msgs::PoseStamped &pose, const std::string &end_effector_name)
  {
    // get all poses in panda_link_0 frame
    geometry_msgs::PoseStamped pose_tf = tf_buffer_.transform(pose, "panda_link0", ros::Duration(1.0));

    geometry_msgs::PoseStamped ee_pose = make_pose_stamped(0, 0, 0, 0, 0, 0, end_effector_name);
    ee_pose = tf_buffer_.transform(ee_pose, "panda_link0", ros::Duration(1.0));

    geometry_msgs::PoseStamped tcp_pose = make_pose_stamped(0, 0, 0, 0, 0, 0, "panda_hand_tcp");
    tcp_pose = tf_buffer_.transform(tcp_pose, "panda_link0", ros::Duration(1.0));

    // compensate for tcp to ee offset
    geometry_msgs::TransformStamped tf_tcp_ee;
    tf_tcp_ee.header.frame_id = "panda_link0";

    // difference in translation
    tf_tcp_ee.transform.translation.x = tcp_pose.pose.position.x - ee_pose.pose.position.x;
    tf_tcp_ee.transform.translation.y = tcp_pose.pose.position.y - ee_pose.pose.position.y;
    tf_tcp_ee.transform.translation.z = tcp_pose.pose.position.z - ee_pose.pose.position.z;

    // difference in rotation
    tf2::Quaternion ee_quat = tf2::Quaternion(ee_pose.pose.orientation.x, ee_pose.pose.orientation.y, ee_pose.pose.orientation.z, ee_pose.pose.orientation.w);
    tf2::Quaternion tcp_quat = tf2::Quaternion(tcp_pose.pose.orientation.x, tcp_pose.pose.orientation.y, tcp_pose.pose.orientation.z, tcp_pose.pose.orientation.w);
    tf_tcp_ee.transform.rotation = tf2::toMsg(ee_quat * tcp_quat.inverse());

    tf2::doTransform(pose_tf, pose_tf, tf_tcp_ee);

    return pose_tf;
  }

  inline bool FrankaInterface::has_transform_changed(const std::string &sourceFrame, const std::string &targetFrame)
  {
    // Get the current time
    const ros::Time currentTime = ros::Time::now();

    // Get the transform 2 seconds ago
    const ros::Time pastTime = currentTime - ros::Duration(2.0);

    // Lookup the past transform
    geometry_msgs::TransformStamped pastTransform = tf_buffer_.lookupTransform(
        targetFrame, sourceFrame, pastTime, ros::Duration(1.0));

    // Lookup the current transform
    geometry_msgs::TransformStamped currentTransform = tf_buffer_.lookupTransform(
        targetFrame, sourceFrame, currentTime, ros::Duration(1.0));

    // Define a tolerance threshold for comparison
    const double tolerance = 0.001; // Adjust the threshold as needed

    // Compare the translation and rotation components
    double translationDiff = std::abs(
                                 pastTransform.transform.translation.x - currentTransform.transform.translation.x) +
                             std::abs(pastTransform.transform.translation.y - currentTransform.transform.translation.y) +
                             std::abs(pastTransform.transform.translation.z - currentTransform.transform.translation.z);

    double rotationDiff = std::abs(
                              pastTransform.transform.rotation.x - currentTransform.transform.rotation.x) +
                          std::abs(pastTransform.transform.rotation.y - currentTransform.transform.rotation.y) +
                          std::abs(pastTransform.transform.rotation.z - currentTransform.transform.rotation.z) +
                          std::abs(pastTransform.transform.rotation.w - currentTransform.transform.rotation.w);

    // Compare the translations and rotations with tolerance
    if (translationDiff > tolerance || rotationDiff > tolerance)
    {
      return true;
    }
    return false;
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

  void FrankaInterface::activate_collision_check()
  {
    planning_scene_interface_.applyCollisionObjects(default_collision_objects_);
    planning_scene_interface_.applyCollisionObjects(custom_collision_objects_);
  }

  void FrankaInterface::deactivate_collision_check()
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

} // namespace franka_interface
```


-------------------------------

Updated on 2023-07-11 at 08:37:05 +0200
```


-------------------------------

Updated on 2023-07-27 at 16:29:38 +0200
