---
title: classfranka________________interface________1________1LinPlanningFailedIncomplete____8md__8md_8md.md

---

# classfranka________________interface________1________1LinPlanningFailedIncomplete____8md__8md_8md.md






## Source code

```markdown
---
title: classfranka________interface____1____1LinPlanningFailedIncomplete__8md_8md.md

---

# classfranka________interface____1____1LinPlanningFailedIncomplete__8md_8md.md






## Source code

```markdown
---
title: classfranka____interface__1__1LinPlanningFailedIncomplete_8md.md

---

# classfranka____interface__1__1LinPlanningFailedIncomplete_8md.md






## Source code

```markdown
---
title: classfranka__interface_1_1LinPlanningFailedIncomplete.md

---

# classfranka__interface_1_1LinPlanningFailedIncomplete.md






## Source code

```markdown
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
