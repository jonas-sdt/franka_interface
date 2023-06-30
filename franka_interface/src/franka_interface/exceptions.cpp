#include "franka_interface/exceptions.hpp"

namespace franka_interface {  

  PlanningFailed::PlanningFailed(moveit::core::MoveItErrorCode error_code)
   : error_code_(error_code)
  {}

  const char * PlanningFailed::what() const noexcept
  {
    std::ostringstream oss;
    oss << "PTP planning failed because of " << moveit::core::MoveItErrorCode::toString(error_code_);
    return oss.str().c_str();
  }
  
  LinPlanningFailedIncomplete::LinPlanningFailedIncomplete(geometry_msgs::PoseStamped pose, double percentage)
   : pose_(pose), percentage_(percentage)
  {}

  const char* LinPlanningFailedIncomplete::what() const noexcept {
    std::ostringstream oss;
    oss << "LIN planning has only covered " << (int)(this->percentage_ * 100) << "\% and therefore failed. Goal Point was: " << this->pose_;
    return oss.str().c_str();
  }

  ExecutionFailed::ExecutionFailed(const char * message)
   : message_(message)
  {}

  const char * ExecutionFailed::what() const noexcept
  {
    return this->message_;
  }
} // namespace franka_interface
