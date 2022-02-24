![](./images/Picto+STEREOLABS_Black.jpg)

# Stereolabs ZED Camera - ROS Noetic Ninjemis Integration

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, pose information and supports the use of multiple ZED cameras.

[More information](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html)

**Note:** The `zed_interfaces` package has been removed from this repository and moved to its own [`zed-ros-interfaces` repository](https://github.com/stereolabs/zed-ros-interfaces) for allowing better integration of the ZED Wrapper on remote ground stations that do not require the full package to be installed. To update your repository please follow the [new update instructions](https://github.com/stereolabs/zed-ros-wrapper#update-the-repository). For more information please read issue [#750](https://github.com/stereolabs/zed-ros-wrapper/issues/750).

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com/developers/)
- [Install](#build-the-program) the ZED ROS wrapper
- For more information, check out our [ROS documentation](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/introduction.html). If you want to customize the wrapper, check the [ZED API documentation](https://www.stereolabs.com/developers/documentation/API/)

### Prerequisites

- Ubuntu 20.04
- [ZED SDK **≥ 3.7**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)

or

- Ubuntu 18.04
- [ZED SDK **≥ 3.7**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

### Build the repository

The zed_ros_wrapper is a catkin package. It depends on the following ROS packages:

   - nav_msgs
   - tf2_geometry_msgs
   - message_runtime
   - catkin
   - roscpp
   - stereo_msgs
   - rosconsole
   - robot_state_publisher
   - urdf
   - sensor_msgs
   - image_transport
   - roslint
   - diagnostic_updater
   - dynamic_reconfigure
   - tf2_ros
   - message_generation
   - nodelet

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ cd ~/catkin_ws/src
    $ git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -r -y
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    $ source ./devel/setup.bash

#### Update the local repository

To update the repository to the latest release you must use the following command to retrieve the latest commits of `zed-ros-wrapper` and of all the submodules:

    $ git checkout master # if you are not on the main branch  
    $ git pull --recurse-submodules # update recursively all the submodules

Remember to always clean the cache of your catkin workspace before compiling with the `catkin_make` command to be sure that everything will work as expected:

    $ roscd
    $ cd ..
    $ rm -rf build
    $ rm -rf devel
    $ catkin_make -DCMAKE_BUILD_TYPE=Release

### Run the ZED wrapper

To launch the ZED node use

ZED camera:

    $ roslaunch zed_wrapper zed.launch
   
ZED Mini camera:

    $ roslaunch zed_wrapper zedm.launch
   
ZED2 camera:

    $ roslaunch zed_wrapper zed2.launch

ZED2i camera:

    $ roslaunch zed_wrapper zed2i.launch    

 To select the ZED from its serial number:
 
     $ roslaunch zed_wrapper zed.launch serial_number:=1010 #replace 1010 with the actual SN

### Rviz visualization
Example launch files to start a pre-configured Rviz environment to visualize the data of ZED, ZED Mini and ZED 2 cameras are provided in the [`zed-ros-examples` repository](https://github.com/stereolabs/zed-ros-examples/tree/master/zed_display_rviz)
    
### SVO recording
[SVO recording](https://www.stereolabs.com/docs/video/#video-recording) can be started and stopped while the ZED node is running using the service `start_svo_recording` and the service `stop_svo_recording`.
[More information](https://www.stereolabs.com/docs/ros/zed_node/#services)

### Object Detection
The SDK v3.0 introduces the Object Detection and Tracking module. **The Object Detection module is available only with a ZED 2 camera**. 

The Object Detection can be enabled *automatically* when the node start setting the parameter `object_detection/od_enabled` to `true` in the file `zed2.yaml`.

The Object Detection can be enabled/disabled *manually* calling the services `start_object_detection` and `stop_object_detection`.

### Spatial Mapping
The Spatial Mapping can be enabled automatically when the node start setting the parameter `mapping/mapping_enabled` to `true` in the file `common.yaml`.
The Spatial Mapping can be enabled/disabled manually calling the services `start_3d_mapping` and `stop_3d_mapping`.

### Diagnostic
The ZED node publishes diagnostic information that can be used by the robotic system using a [diagnostic_aggregator node](http://wiki.ros.org/diagnostic_aggregator).

With the `rqt` plugin `Runtime monitor`, it is possible to retrieve all the diagnostic information, checking that the node 
is working as expected.

### 2D mode
For robots moving on a planar surface it is possible to activate the "2D mode" (parameter `tracking/two_d_mode` in `common.yaml`). 
The value of the coordinate Z for odometry and pose will have a fixed value (parameter `tracking/fixed_z_value` in `common.yaml`). 
Roll and pitch and relative velocities will be fixed to zero.

## Examples and Tutorials
Examples and tutorials are provided to better understand how to use the ZED wrapper and how to integrate it in the ROS framework.
See the [`zed-ros-examples` repository](https://github.com/stereolabs/zed-ros-examples)

### Examples
Alongside the wrapper itself and the Rviz display, a few examples are provided to interface the ZED with other ROS packages :

- [RTAB-Map](http://introlab.github.io/rtabmap/): See [zed_rtabmap_example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_rtabmap_example/README.md)
- ROS Nodelet, `depthimage_to_laserscan`: See [zed_nodelet_example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_nodelet_example/README.md)
- AR Track Alvar: See [zed_ar_track_alvar_example](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_ar_track_alvar_example/README.md)

### Tutorials

A few tutorials are provided to understand how to use the ZED node in the ROS environment :

 - [Image subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_video_sub_tutorial/README.md)
 - [Depth subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_depth_sub_tutorial/README.md)
 - [Tracking subscription tutorial](https://github.com/stereolabs/zed-ros-examples/tree/master/tutorials/zed_tracking_sub_tutorial/README.md) 
 - [Sensors data subscription tutorial](https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_sensors_sub_tutorial/README.md) 
 - [Object detection subscription tutorial](https://github.com/stereolabs/zed-ros-examples/blob/master/tutorials/zed_obj_det_sub_tutorial/README.md) 

