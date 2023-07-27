---
title: franka_interface::PlanningFailed
summary: Exception thrown when a planning operation fails. 

---

# franka_interface::PlanningFailed



Exception thrown when a planning operation fails. 


`#include <exceptions.hpp>`

Inherits from exception

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[PlanningFailed](Classes/classfranka__interface_1_1PlanningFailed.md#function-planningfailed)**(moveit::core::MoveItErrorCode error_code)<br>Exception thrown when a planning operation fails.  |
| const char * | **[what](Classes/classfranka__interface_1_1PlanningFailed.md#function-what)**() const override |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| moveit::core::MoveItErrorCode | **[error_code_](Classes/classfranka__interface_1_1PlanningFailed.md#variable-error-code-)** <br>The error code that was returned by the planning operation.  |

## Public Functions Documentation

### function PlanningFailed

```cpp
PlanningFailed(
    moveit::core::MoveItErrorCode error_code
)
```

Exception thrown when a planning operation fails. 

**Parameters**: 

  * **error_code** The error code that was returned by the planning operation. 


### function what

```cpp
const char * what() const override
```


## Public Attributes Documentation

### variable error_code_

```cpp
moveit::core::MoveItErrorCode error_code_;
```

The error code that was returned by the planning operation. 

-------------------------------

Updated on 2023-07-27 at 16:29:37 +0200