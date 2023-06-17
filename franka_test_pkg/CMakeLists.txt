cmake_minimum_required(VERSION 3.0.2)
project(franka_example_pkg)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  franka_interface
  roscpp
)


###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS franka_interface roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(arm_example_node src/franka_arm_example_node.cpp)
add_executable(gripper_example_node src/franka_gripper_example_node.cpp)
add_executable(desk_height_test_node src/franka_desk_height_test.cpp)

target_link_libraries(arm_example_node
  ${catkin_LIBRARIES}
)
target_link_libraries(gripper_example_node
  ${catkin_LIBRARIES}
)
target_link_libraries(desk_height_test_node
  ${catkin_LIBRARIES}
)
