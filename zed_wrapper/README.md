# Stereolabs ZED Camera - ROS Integration

This package lets you use the ZED stereo cameras with ROS. It outputs the camera left and right images, depth map, point cloud, pose information and supports the use of multiple ZED cameras.
The ZED ROS wrapper is compatible with ZED, ZED Mini and ZED2 camera models.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/)
- [Install](#build-the-program) the ZED ROS wrapper.
- For more information, check out our [ROS documentation](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html) or our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper). If you want to customize the wrapper, check the [ZED API documentation](https://www.stereolabs.com/developers/documentation/API/).

### Prerequisites

- Ubuntu 16.04 or newer
- [ZED SDK **≥ 3.0**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) or newer

### Build the program

The *zed_ros_wrapper* is a catkin package. It depends on the following ROS packages:

- tf2_ros
- tf2_geometry_msgs
- nav_msgs
- roscpp
- rosconsole
- sensor_msgs
- stereo_msgs
- image_transport
- dynamic_reconfigure
- nodelet
- diagnostic_updater
- urdf
- message_generation
- roslint
- robot_state_publisher
- message_runtime

Open a terminal and clone the package in your catkin workspace:
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/stereolabs/zed-ros-wrapper.git
    $ cd ../

Automatically satisfy all the dependencies
    $ rosdep install --from-paths src --ignore-src -r -y
**Note**: This command magically installs all the packages that the packages in your catkin workspace depend upon
but are missing on your computer.

Compile the package
    $ catkin_make
    $ source ./devel/setup.bash

### Run the program

To launch ZED node, use:

*ZED*
    $ roslaunch zed_wrapper zed.launch
*ZED Mini*
    $ roslaunch zed_wrapper zedm.launch
*ZED 2*
    $ roslaunch zed_wrapper zed2.launch

 To select the ZED from its serial number

    $ roslaunch zed_wrapper zed.launch serial_number:=1010 

**Note**: replace 1010 with the actual SN

If you want to use the `ZEDWrapperNodelet` with an external nodelet manager follow the [`zed_nodelet_example`](https://github.com/stereolabs/zed-ros-wrapper/tree/master/examples/zed_nodelet_example) approach

### Diagnostic
The ZED node publishes diagnostic information that can be used by the robotic system using a [diagnostic_aggregator node](http://wiki.ros.org/diagnostic_aggregator).

With the `rqt` plugin `Runtime monitor`, it is possible to retrieve all the diagnostic information, checking that the node is working as expected:

![](../images/rqt_diagnostic.jpg)

A brief explanation of each field:

  -  `Component`: name of the diagnostic component
  -  `Message`: summary of the status of the ZED node
  -  `HardwareID`: Model of the ZED camera and its serial number
  -  `Capture`: grabbing frequency (if video or depth data are subscribed) and the percentage respect to the camera frame rate
  -  `Processing time`: time in seconds spent to elaborate data and the time limit to achieve max frame rate
  -  `Depth status`: indicates if the depth processing is performed
  -  `Point Cloud`: point cloud publishing frequency (if there is at least a subscriber) and the percentage respect to the camera frame rate
  -  `Floor Detection`: if the floor detection is enabled, indicates if the floor has been detected and the camera position correctly initialized
  -  `Tracking status`: indicates the status of the tracking, if enabled
  -  `IMU`: the publishing frequency of the IMU topics, if the camera is the ZED Mini and there is at least a subscriber


[More](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)




