# Stereolabs ZED Camera - ROS Integration

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, odometry information and supports the use of multiple ZED cameras.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com)
- Download the ZED ROS wrapper [here](https://github.com/stereolabs/zed-ros-wrapper/archive/master.zip).
- For more information, check out our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper) or [blog post](https://www.stereolabs.com/blog/index.php/2015/09/07/use-your-zed-camera-with-ros/) and read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/)

### Prerequisites

- Ubuntu 16.04
- [ZED SDK **2.1**](https://www.stereolabs.com/developers/) and its dependencies ([OpenCV](http://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html), [CUDA](https://developer.nvidia.com/cuda-downloads))
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Point Cloud Library (PCL)](https://github.com/PointCloudLibrary/pcl)

### Build the program

The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

   - tf2_ros
   - nav_msgs
   - roscpp
   - rosconsole
   - sensor_msgs
   - opencv
   - image_transport
   - dynamic_reconfigure
   - urdf

Place the package folder `zed_wrapper` in your catkin workspace source folder `~/catkin_ws/src`.

Open a terminal and build the package:

    cd ~/catkin_ws/
    catkin_make
    source ./devel/setup.bash

### Run the program

Open a terminal and launch the wrapper:

    roslaunch zed_wrapper zed.launch

Open a second terminal to display the rectified left color image (reference view):

    rosrun image_view image_view image:=/camera/rgb/image_rect_color


[More](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)
