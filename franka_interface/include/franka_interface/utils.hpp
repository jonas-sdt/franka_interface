#ifndef FRANKA_INTERFACE_UTILS_HPP
#define FRANKA_INTERFACE_UTILS_HPP

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include <iostream>
#include <vector>

namespace franka_interface
{

    typedef std::vector<double> JointPositions;

    /**
     * \brief create a joint state goal message
     * \param q1 joint 1 position in radians
    */
    JointPositions make_joint_state_goal(long double q1, long double q2, long double q3, long double q4, long double q5, long double q6, long double q7);
    
    /**
     * \brief create a joint state goal message
     * \param q1 joint 1 position in radians
    */
    JointPositions make_joint_state_goal(double q1, double q2, double q3, double q4, double q5, double q6, double q7);

    /**
    * \brief create a pose stamped message
    * \param x x position in meters
    * \param y y position in meters
    * \param z z position in meters
    * \param qx x component of the quaternion
    * \param qy y component of the quaternion
    * \param qz z component of the quaternion
    * \param qw w component of the quaternion
    * \param frame_id frame id of the pose
    */
    geometry_msgs::PoseStamped make_pose_stamped(double x, double y, double z, double qx, double qy, double qz, double qw, std::string frame_id);

    /**
     * \brief create a pose stamped message
     * \param x x position in meters
     * \param y y position in meters
     * \param z z position in meters
     * \param roll roll angle in radians
     * \param pitch pitch angle in radians
     * \param yaw yaw angle in radians
     * \param frame_id frame id of the pose
     */
    geometry_msgs::PoseStamped make_pose_stamped(double x, double y, double z, double roll, double pitch, double yaw, std::string frame_id);

    /**
     * \brief create a pose message
     * \param x x position in meters
     * \param y y position in meters
     * \param z z position in meters
     * \param qx x component of the quaternion
     * \param qy y component of the quaternion
     * \param qz z component of the quaternion
     * \param qw w component of the quaternion
     */
    geometry_msgs::Pose make_pose(double x, double y, double z, double qx, double qy, double qz, double qw);

    /**
     * \brief create a pose message
     * \param x x position in meters
     * \param y y position in meters
     * \param z z position in meters
     * \param roll roll angle in radians
     * \param pitch pitch angle in radians
     * \param yaw yaw angle in radians
     */
    geometry_msgs::Pose make_pose(double x, double y, double z, double roll, double pitch, double yaw);

    /**
     * \brief create a string from a pose stamped message
     * \param pose_stamped pose stamped message
     */
    std::string pose_stamped_to_string(const geometry_msgs::PoseStamped & pose_stamped);

    /**
     * \brief create a string from a pose message
     * \param pose pose message
     */
    std::string pose_to_string(const geometry_msgs::Pose & pose);

    std::ostream& operator<<(std::ostream& os, const geometry_msgs::PoseStamped& pose_stamped);

    std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& pose);

    long double operator"" _m(unsigned long long int value);

    long double operator"" _m(long double value);

    long double operator"" _cm(unsigned long long int value);

    long double operator"" _cm(long double value);

    long double operator"" _mm(unsigned long long int value);

    long double operator"" _mm(long double value);

    long double operator"" _deg(unsigned long long int value);

    long double operator"" _deg(long double value);

    long double operator"" _rad(unsigned long long int value);

    long double operator"" _rad(long double value);

} // namespace franka_interface

#endif // FRANKA_INTERFACE_UTILS_HPP