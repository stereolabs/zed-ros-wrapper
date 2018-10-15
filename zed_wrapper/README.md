# Stereolabs ZED Camera - ROS Integration

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, pose information and supports the use of multiple ZED cameras.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/)
- [Install](#build-the-program) the ZED ROS wrapper.
- For more information, check out our [ROS documentation](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) or our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper). If you want to customize the wrapper, check the [ZED API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Ubuntu 16.04
- [ZED SDK **≥ 2.3**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Build the program

The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

   - tf2_ros
   - tf2_geometry_msgs
   - nav_msgs
   - roscpp
   - rosconsole
   - sensor_msgs
   - stereo_msgs
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

To launch ZED node, use:

    $ roslaunch zed_wrapper zed.launch

**Note**: Remember to change the parameter `camera_model` to `0` if you are using a **ZED** or to `1` if you are using a **ZED Mini**

 To select the ZED from its serial number

    $ roslaunch zed_wrapper zed.launch serial_number:=1010 

**Note**: replace 1010 with the actual SN

If you want to use the `ZEDWrapperNodelet` with an external nodelet manager follow the `zed_nodelet_example` approach

[More](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)
