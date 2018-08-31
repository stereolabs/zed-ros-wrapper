# Stereolabs ZED Camera - ROS Navigation Tutorial

This package illustrates how to integrate the ZED [Terrain Mapping](https://www.stereolabs.com/developers/documentation/API/) module with the "[ROS Navigation](http://wiki.ros.org/navigation?distro=kinetic)" stack. 
The tutorial is mainly directed to ground robots and uses the [Turtlebot2](https://www.turtlebot.com/turtlebot2/) as robotic platform.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/)
- [Install](#build-the-program) the ZED ROS wrapper.
- For more information, check out our [ROS documentation](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) or our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper). If you want to customize the wrapper, check the [ZED API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Ubuntu 16.04
- [ZED SDK **≥ 2.7**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Turtlebot2](https://www.turtlebot.com/turtlebot2/) tutorial robotic platform

### Execution
- Bringup the TurtleBot2 robot:

  `$ roslaunch turtlebot_bringup minimal.launch --screen`
  
- Start the ZED SDK and the mapping (SLAM) using the command

  `$ roslaunch zed_navigation_tutorial zed_navigation_demo.launch`
  
  The ZED nodelet is started as in the "zed_nodelet_example" to generate a "/zed/virt_scan" topic that can be used to easily detect obstacles.
  
  The [`move_base`](http://wiki.ros.org/move_base) node is also started to initialize the ROS Navigation interface.
  
- Start the visualization interface using the command

  `$ roslaunch zed_navigation_tutorial zed_display_navigation.launch`
  
  we suggest to execute this command on another machine on the same local network since simoultaneous visualization and mapping on the same system require a huge amount of computational power.

- To set a **goal** to be reached by the robot on Rviz select `2D nav goal` on the upper toolbar and click on a point of the 3D view to set the final position and orientation

  [TODO add image]


[More](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

https://husarion.com/tutorials/ros-tutorials/6-slam-navigation/
