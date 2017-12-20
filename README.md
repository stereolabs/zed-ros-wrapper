# Stereolabs ZED Camera - ROS Integration

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, odometry information and supports the use of multiple ZED cameras.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/)
- [Install](#build-the-program) the ZED ROS wrapper.
- For more information, check out our [ROS documentation](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) or our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper). If you want to customize the wrapper, check the [ZED API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Ubuntu 16.04
- [ZED SDK **≥ 2.3**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)

### Build the program

The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

   - tf2_ros
   - tf2_geometry_msgs
   - nav_msgs
   - roscpp
   - rosconsole
   - sensor_msgs
   - opencv
   - image_transport
   - dynamic_reconfigure
   - urdf


Open a terminal and build the package:

    cd ~/catkin_ws/src
    git clone https://github.com/stereolabs/zed-ros-wrapper.git
    cd ../
    catkin_make
    source ./devel/setup.bash

### Run the program

To launch the wrapper along with an Rviz preview, open a terminal and launch:

    roslaunch zed_wrapper display.launch # by default open a ZED

or

    roslaunch zed_wrapper display_zedm.launch # open a ZED Mini


To launch the wrapper without Rviz, use:

    roslaunch zed_wrapper zed.launch

 To select the ZED from its serial number

    roslaunch zed_wrapper zed.launch serial_number:=1010 #replace 1010 with the actual SN

[More](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)
