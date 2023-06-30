#ifndef FRANKA_INTERFACE_EXCEPTIONS_HPP
#define FRANKA_INTERFACE_EXCEPTIONS_HPP

#include <algorithm>
#include <cmath>
#include <exception>
#include <stdexcept>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "moveit/moveit_cpp/moveit_cpp.h"
#include <map>
#include <string>
#include "franka_interface/utils.hpp"
#include <sstream>

namespace franka_interface {

  class PlanningFailed : public std::exception
  {
  public:
    moveit::core::MoveItErrorCode error_code_;
    const char * what() const noexcept override;
    PlanningFailed(moveit::core::MoveItErrorCode error_code);
  };

  class LinPlanningFailedIncomplete : public std::exception
  {
  public:
    const char * what() const noexcept override;
    geometry_msgs::PoseStamped pose_;
    double percentage_;
    LinPlanningFailedIncomplete(geometry_msgs::PoseStamped pose, double percentage);
  };

  class ExecutionFailed : public std::exception
  {
    const char * message_;
  public:
    ExecutionFailed(const char * message);
    const char * what() const noexcept override;
  };

} // namespace franka_interface

#endif // FRANKA_INTERFACE_EXCEPTIONS_HPP