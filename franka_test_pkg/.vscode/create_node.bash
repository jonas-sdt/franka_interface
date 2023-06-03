#!/bin/bash

FILE=src/$2.cpp
if test -f "$FILE"; then
    echo "Node \"$2\" already exists."
    exit 1
fi

touch $FILE

echo "#include \"ros/ros.h\"

int main(int argc, char** argv)
{
    ros::init(argc, argv, \"$2\");
    ros::NodeHandle nh;

    // Put your code here

}" > $FILE

echo "  add_executable($2 $FILE)
  target_link_libraries($2 \${catkin_LIBRARIES})" >> CMakeLists.txt

echo "Created node \"$2\" in package \"$1\"."

