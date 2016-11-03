# Stereolabs ZED Camera - ROS Integration

This package lets you use the ZED stereo camera with ROS. It outputs the camera left and right images, depth map, point cloud, odometry information and supports the use of multiple ZED cameras.

## Getting started

- First, download the latest version of the ZED SDK on [stereolabs.com](https://www.stereolabs.com)
- Download the ZED ROS wrapper [here](https://github.com/stereolabs/zed-ros-wrapper/archive/master.zip).
- For more information, check out our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper) or [blog post](https://www.stereolabs.com/blog/index.php/2015/09/07/use-your-zed-camera-with-ros/) and read the ZED [API documentation](https://www.stereolabs.com/developers/documentation/API/)

### Prerequisites

- Ubuntu 16.04
- [ZED SDK](https://www.stereolabs.com/developers/) and its dependencies ([OpenCV](http://docs.opencv.org/3.1.0/d7/d9f/tutorial_linux_install.html), [CUDA](https://developer.nvidia.com/cuda-downloads))
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

## Features

### Topics

#### Left camera
   - */camera/rgb/image_rect_color* : `Color rectified image (left RGB image by default).`
   - */camera/rgb/image_raw_color* : `Color unrectified image (left RGB image by default).`
   - */camera/rgb/camera_info* : `Camera calibration data.`
   - */camera/left/image_rect_color* : `Color rectified left image.`
   - */camera/left/image_raw_color* : `Color unrectified left image.`
   - */camera/left/camera_info* : `Left camera calibration data.`

#### Right camera
  - */camera/right/image_rect_color* : `Color rectified right image.`
  - */camera/right/image_raw_color* : `Color unrectified right image.`
  - */camera/right/camera_info* : `Right camera calibration data.`

#### Depth and point cloud
   - */camera/depth/depth_registered* : `Depth map image registered on left image (by default 32 bits float, in meters).`
   - */camera/point_cloud/cloud_registered* : `Registered color point cloud.`

#### Visual odometry
   - */camera/odom* : `Absolute 3D position and orientation relative to zed_initial_frame.`

All topics have their *id* published.

### Launch file parameters

Specify your launch parameters in the zed_camera.launch file available  [here](https://github.com/stereolabs/zed-ros-wrapper/tree/master/launch).


 Parameter                    |           Description                                       |              Value          
------------------------------|-------------------------------------------------------------|-------------------------    
 svo_file                     | Specify SVO filename                                                 | Path to an SVO file         
 resolution                   | Select ZED camera resolution                                       | '0': HD2K, '1': HD1080, '2': HD720, '3': VGA
  frame_rate                   | Set ZED camera video framerate | int                      
  sensing_mode                 | Select depth sensing mode                                          | '0': FILL, '1': STANDARD                   
 quality                      | Select depth map quality                                       | '0': NONE, '1': PERFORMANCE, '2': MEDIUM, '3': QUALITY
 openni_depth_mode            | Convert 32bit depth in meters to 16bit in millimeters                       | '0': 32bit float meters, '1': 16bit uchar millimeters   
 zed_id                    | Select a ZED camera by its ID. ID are assigned by Ubuntu. Useful when multiple cameras are connected. ID is ignored if an SVO path is specified.                      | int, default '0'
 gpu_id                   | Select a GPU device for depth computation | int, default '-1' (best device found)                     

Topic names can be customized in the launch file.

### Dynamic parameter

The only parameter that can be configured dynamically is the depth map confidence threshold (using `rqt`). The confidence value is mapped between 0 (high confidence threshold, sparse data) and 100 (low confidence threshold, dense data).


## Transform frame
The ZED ROS wrapper broadcasts multiple coordinate frames that each provide information about the camera position and orientation.

-   `zed_initial_frame` is the starting position and orientation of ZED left camera. It is the origin of the `map` frame.
-   `zed_current_frame` is the current position and orientation of ZED left camera determined by visual odometry.
-   `ZED_left_camera` is the position and orientation of ZED left camera. It is the same as `zed_current_frame`.
-   `ZED_right_camera` is the position and orientation of ZED right camera.
-   `ZED_center` is the position and orientation of ZED center of gravity.


For RVIZ compatibilty, the root frame is `map`.
```
map
└─zed_initial_frame
  └─zed_current_frame
    └─ZED_left_camera    
      │ ZED_right_camera
      │ ZED_center
```



## Using RVIZ

rviz (ROS visualization) is a 3D visualizer for displaying sensor data and state information. Using rviz, you can visualize ZED left and right images, depth, point cloud and 3D trajectory.

To learn how to use rviz to display ZED data, check out our [ROS wiki](http://wiki.ros.org/zed-ros-wrapper).


## Using multiple ZED with ROS

It is possible to use multiple ZED cameras with ROS. Simply launch [zed_multi_cam](https://github.com/stereolabs/zed-ros-wrapper/blob/master/launch/zed_multi_cam.launch) using the `roslaunch` command:

    roslaunch zed_wrapper zed_multi_cam.launch



#### Assigning a GPU to a camera

To improve performance, you can specify in the [launch file ](https://github.com/stereolabs/zed-ros-wrapper/blob/master/launch/zed_multi_gpu.launch) the `gpu_id` of the graphics card that will be used for depth computation. By default, (-1) will select the GPU with the highest number of CUDA cores. When using multiple ZED, you can assign specific GPUs to different cameras.


## Limitations

#### Performance

This wrapper lets you quickly prototype applications and interface the ZED with other sensors and packages available in ROS.
However, the ROS layer introduces significant latency and a performance hit. If performance is a major concern for your application, please consider using the ZED SDK library.

#### Using multiple ZED

The ZED camera uses the full USB 3.0 bandwidth to output video. When using multiple ZED, you may need to reduce camera framerate and resolution to avoid corrupted frames (green or purple frames). You can also use multiple GPUs to load-balance computations and improve performance.
