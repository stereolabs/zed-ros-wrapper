![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS Integration

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, pose information and supports the use of multiple ZED cameras.

[More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

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
   - diagnostic_updater

Open a terminal and build the package:

    cd ~/catkin_ws/src
    git clone https://github.com/stereolabs/zed-ros-wrapper.git
    cd ../
    catkin_make
    source ./devel/setup.bash

### Run the program

To launch the wrapper [along with an Rviz preview](./zed_display_rviz), open a terminal and launch:

    $ roslaunch zed_display_rviz display.launch # by default open a ZED

or

    $ roslaunch zed_display_rviz display_zedm.launch # open a ZED Mini


To launch the wrapper without Rviz, use:

    $ roslaunch zed_wrapper zed.launch

 To select the ZED from its serial number

    $ roslaunch zed_wrapper zed.launch serial_number:=1010 #replace 1010 with the actual SN
    
### SVO recording
[SVO recording](https://www.stereolabs.com/docs/video/#video-recording) can be started and stopped while the ZED node is running using the service `start_svo_recording` and the service `stop_svo_recording`.
[More information](https://www.stereolabs.com/docs/ros/zed_node/#services)

### Diagnostic
The ZED node publishes diagnostic information that can be used by the robotic system using a [diagnostic_aggregator node](http://wiki.ros.org/diagnostic_aggregator).

With the `rqt` plugin `Runtime monitor`, it is possible to retrieve all the diagnostic information, checking that the node 
is working as expected.

### 2D mode
For robots moving on a planar surface it is possible to activate the "2D mode" (parameter `two_d_mode`). The value of the coordinate Z for odometry and pose will have a fixed value (parameter `fixed_z_value`). Roll and pitch will be fixed to zero, like relative velocities.

### Examples

Alongside the wrapper itself and the Rviz display, a few examples are provided to interface the ZED with other ROS packages :

- [RTAB-Map](http://introlab.github.io/rtabmap/) : See [zed_rtabmap_example](./examples/zed_rtabmap_example)
- ROS Nodelet, `depthimage_to_laserscan` : See [zed_nodelet_example](./examples/zed_nodelet_example)

### Tutorials

A few tutorials are provided to understand how to use the ZED node in the ROS environment :

- Video subscribing : See [zed_video_sub_tutorial](./tutorials/zed_video_sub_tutorial)
- Depth subscribing : See [zed_depth_sub_tutorial](./tutorials/zed_depth_sub_tutorial)
- Positional Tracking subscribing : See [zed_tracking_sub_tutorial](./tutorials/zed_tracking_sub_tutorial)
