#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "tf2/LinearMath/Quaternion.h"
#include <iostream>

namespace franka_interface
{

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
    geometry_msgs::PoseStamped make_pose_stamped(float x, float y, float z, float qx, float qy, float qz, float qw, std::string frame_id)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.pose.position.x = x;
        pose_stamped.pose.position.y = y;
        pose_stamped.pose.position.z = z;
        pose_stamped.pose.orientation.x = qx;
        pose_stamped.pose.orientation.y = qy;
        pose_stamped.pose.orientation.z = qz;
        pose_stamped.pose.orientation.w = qw;
        pose_stamped.header.frame_id = frame_id;
        return pose_stamped;
    }

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
    geometry_msgs::PoseStamped make_pose_stamped(float x, float y, float z, float roll, float pitch, float yaw, std::string frame_id)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return make_pose_stamped(x, y, z, q.x(), q.y(), q.z(), q.w(), frame_id);
    }

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
    geometry_msgs::Pose make_pose(float x, float y, float z, float qx, float qy, float qz, float qw)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = z;
        pose.orientation.x = qx;
        pose.orientation.y = qy;
        pose.orientation.z = qz;
        pose.orientation.w = qw;
        return pose;
    }

    /**
     * \brief create a pose message
     * \param x x position in meters
     * \param y y position in meters
     * \param z z position in meters
     * \param roll roll angle in radians
     * \param pitch pitch angle in radians
     * \param yaw yaw angle in radians
     */
    geometry_msgs::Pose make_pose(float x, float y, float z, float roll, float pitch, float yaw)
    {
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        return make_pose(x, y, z, q.x(), q.y(), q.z(), q.w());
    }

    /**
     * \brief create a string from a pose stamped message
     * \param pose_stamped pose stamped message
     */
    std::string pose_stamped_to_string(const geometry_msgs::PoseStamped & pose_stamped)
    {
        std::string str = "PoseStamped: ";
        str += "frame_id: " + pose_stamped.header.frame_id + ", \n";
        str += "position: (" + std::to_string(pose_stamped.pose.position.x) + ", " + std::to_string(pose_stamped.pose.position.y) + ", " + std::to_string(pose_stamped.pose.position.z) + "), \n";
        str += "orientation: (" + std::to_string(pose_stamped.pose.orientation.x) + ", " + std::to_string(pose_stamped.pose.orientation.y) + ", " + std::to_string(pose_stamped.pose.orientation.z) + ", " + std::to_string(pose_stamped.pose.orientation.w) + ")";
        return str;
    }

    /**
     * \brief create a string from a pose message
     * \param pose pose message
     */
    std::string pose_to_string(const geometry_msgs::Pose & pose)
    {
        std::string str = "Pose: ";
        str += "position: (" + std::to_string(pose.position.x) + ", " + std::to_string(pose.position.y) + ", " + std::to_string(pose.position.z) + "), \n";
        str += "orientation: (" + std::to_string(pose.orientation.x) + ", " + std::to_string(pose.orientation.y) + ", " + std::to_string(pose.orientation.z) + ", " + std::to_string(pose.orientation.w) + ")";
        return str;
    }

    std::ostream& operator<<(std::ostream& os, const geometry_msgs::PoseStamped& pose_stamped)
    {
        os << pose_stamped_to_string(pose_stamped);
        return os;
    }

    std::ostream& operator<<(std::ostream& os, const geometry_msgs::Pose& pose)
    {
        os << pose_to_string(pose);
        return os;
    }

    long double operator"" _m(unsigned long long int value)
    {
        return value;
    }

    long double operator"" _m(long double value)
    {
        return value;
    }

    long double operator"" _cm(unsigned long long int value)
    {
        return value/100.0;
    }

    long double operator"" _cm(long double value)
    {
        return value/100.0;
    }

    long double operator"" _mm(unsigned long long int value)
    {
        return value/1000.0;
    }

    long double operator"" _mm(long double value)
    {
        return value/1000.0;
    }

    long double operator"" _deg(unsigned long long int value)
    {
        return value*M_PI/180.0;
    }

    long double operator"" _deg(long double value)
    {
        return value*M_PI/180.0;
    }

    long double operator"" _rad(unsigned long long int value)
    {
        return value;
    }

    long double operator"" _rad(long double value)
    {
        return value;
    }

} // namespace franka_interface