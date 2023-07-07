---
title: franka_interface::ExecutionFailed
summary: Exception thrown when an execution operation fails. 

---

# franka_interface::ExecutionFailed



Exception thrown when an execution operation fails. 


`#include <exceptions.hpp>`

Inherits from exception

## Public Functions

|                | Name           |
| -------------- | -------------- |
| | **[ExecutionFailed](Classes/classfranka__interface_1_1ExecutionFailed.md#function-executionfailed)**(const char * message)<br>Exception thrown when an execution operation fails.  |
| const char * | **[what](Classes/classfranka__interface_1_1ExecutionFailed.md#function-what)**() const override |

## Public Functions Documentation

### function ExecutionFailed

```cpp
ExecutionFailed(
    const char * message
)
```

Exception thrown when an execution operation fails. 

**Parameters**: 

  * **message** The message that should be returned by [what()](Classes/classfranka__interface_1_1ExecutionFailed.md#function-what). 


### function what

```cpp
const char * what() const override
```


-------------------------------

Updated on 2023-07-07 at 13:14:33 +0200