---
title: franka_interface::LinPlanningFailedIncomplete
summary: Exception thrown when a linear planning operation fails due to the goal pose not being reached. 

---

# franka_interface::LinPlanningFailedIncomplete



Exception thrown when a linear planning operation fails due to the goal pose not being reached. 


`#include <exceptions.hpp>`

Inherits from exception

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[LinPlanningFailedIncomplete](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md#function-linplanningfailedincomplete)**(geometry_msgs::PoseStamped pose, double percentage)<br>Exception thrown when a linear planning operation fails due to the goal pose not being reached.  |
| const char * | **[what](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md#function-what)**() const override |

## Public Attributes

|                | Name           |
| -------------- | -------------- |
| double | **[percentage_](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md#variable-percentage-)** <br>The percentage of the path that planning covered.  |
| geometry_msgs::PoseStamped | **[pose_](Classes/classfranka__interface_1_1LinPlanningFailedIncomplete.md#variable-pose-)** <br>The goal pose that was not reached.  |

## Public Functions Documentation

### function LinPlanningFailedIncomplete

```cpp
LinPlanningFailedIncomplete(
    geometry_msgs::PoseStamped pose,
    double percentage
)
```

Exception thrown when a linear planning operation fails due to the goal pose not being reached. 

**Parameters**: 

  * **pose** The goal pose that was not reached. 
  * **percentage** The percentage of the path that was covered. 


### function what

```cpp
const char * what() const override
```


## Public Attributes Documentation

### variable percentage_

```cpp
double percentage_;
```

The percentage of the path that planning covered. 

### variable pose_

```cpp
geometry_msgs::PoseStamped pose_;
```

The goal pose that was not reached. 

-------------------------------

Updated on 2023-07-07 at 13:14:33 +0200