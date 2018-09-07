cmake_minimum_required(VERSION 2.8.3)
project(zed_depth_sub_tutorial)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

## Declare a catkin package
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

## Build 
add_executable(zed_depth_sub src/zed_depth_sub_tutorial.cpp)
target_link_libraries(zed_depth_sub ${catkin_LIBRARIES})
