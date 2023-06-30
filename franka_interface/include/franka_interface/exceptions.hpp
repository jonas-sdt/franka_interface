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

namespace franka_interface
{

  /**
   * \brief Exception thrown when a planning operation fails.
   */
  class PlanningFailed : public std::exception
  {
  public:
    /**
     * \brief Exception thrown when a planning operation fails.
     *
     * \param error_code The error code that was returned by the planning operation.
     */
    PlanningFailed(moveit::core::MoveItErrorCode error_code);

    const char *what() const noexcept override;

    /**
     * \brief The error code that was returned by the planning operation.
     */
    moveit::core::MoveItErrorCode error_code_;
  };

  /**
   * \brief Exception thrown when a linear planning operation fails due to the goal pose not being reached.
   */
  class LinPlanningFailedIncomplete : public std::exception
  {
  public:
    /**
     * \brief Exception thrown when a linear planning operation fails due to the goal pose not being reached.
     *
     * \param pose The goal pose that was not reached.
     * \param percentage The percentage of the path that was covered.
     */
    LinPlanningFailedIncomplete(geometry_msgs::PoseStamped pose, double percentage);

    const char *what() const noexcept override;

    /**
     * \brief The goal pose that was not reached.
     */
    geometry_msgs::PoseStamped pose_;

    /**
     * \brief The percentage of the path that planning covered.
     */
    double percentage_;
  };

  /**
   * \brief Exception thrown when an execution operation fails.
   */
  class ExecutionFailed : public std::exception
  {
    const char *message_;

  public:
    /**
     * \brief Exception thrown when an execution operation fails.
     *
     * \param message The message that should be returned by what().
     */
    ExecutionFailed(const char *message);

    const char *what() const noexcept override;
  };

} // namespace franka_interface

#endif // FRANKA_INTERFACE_EXCEPTIONS_HPP