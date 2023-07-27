---
title: exceptions____8hpp__8md_8md.md

---

# exceptions____8hpp__8md_8md.md






## Source code

```markdown
---
title: exceptions__8hpp_8md.md

---

# exceptions__8hpp_8md.md






## Source code

```markdown
---
title: exceptions_8hpp.md

---

# exceptions_8hpp.md






## Source code

```markdown
---
title: franka_interface/exceptions.hpp

---

# franka_interface/exceptions.hpp



## Namespaces

| Name           |
| -------------- |
| **[franka_interface](Namespaces/namespacefranka__interface.md)**  |

## Classes

|                | Name           |
| -------------- | -------------- |
| class | **[franka_interface::ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md)** <br>Exception thrown when an execution operation fails.  |
| class | **[franka_interface::LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md)** <br>Exception thrown when a linear planning operation fails due to the goal pose not being reached.  |
| class | **[franka_interface::PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md)** <br>Exception thrown when a planning operation fails.  |




## Source code

```cpp
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

  class PlanningFailed : public std::exception
  {
  public:
    PlanningFailed(moveit::core::MoveItErrorCode error_code);

    const char *what() const noexcept override;

    moveit::core::MoveItErrorCode error_code_;
  };

  class LinPlanningFailedIncomplete : public std::exception
  {
  public:
    LinPlanningFailedIncomplete(geometry_msgs::PoseStamped pose, double percentage);

    const char *what() const noexcept override;

    geometry_msgs::PoseStamped pose_;

    double percentage_;
  };

  class ExecutionFailed : public std::exception
  {
    const char *message_;

  public:
    ExecutionFailed(const char *message);

    const char *what() const noexcept override;
  };

} // namespace franka_interface

#endif // FRANKA_INTERFACE_EXCEPTIONS_HPP
```


-------------------------------

Updated on 2023-07-10 at 09:26:48 +0200
```


-------------------------------

Updated on 2023-07-10 at 09:42:18 +0200
```


-------------------------------

Updated on 2023-07-11 at 08:37:05 +0200
```


-------------------------------

Updated on 2023-07-27 at 16:29:38 +0200
