# zed-ros-wrapper
Ros wrapper for the ZED Stereo Camera SDK

**This sample is designed to work with the ZED stereo camera only and requires the ZED SDK. For more information: https://www.stereolabs.com**

This sample is a wrapper for the ZED library in order to use the ZED Camera with ROS.
You can publish Left+Depth or Left+Right images and camera info on the topics of your choice.

A set of parameters can be specified in the launch file. Two launch files are provided in the launch directory:

       - zed_depth.launch: publish left and depth images with their camera info.
       - zed_stereo.launch: publish left and right images with their camera info.

The zed_depth_stereo_wrapper is a catkin package made to run on ROS Indigo, and depends
on the following ROS packages:
   - roscpp
   - rosconsole
   - sensor_msgs
   - opencv2
   - cv_bridge
   - image_transport

## Build the program

Place the package folder "zed_wrapper" in your catkin workspace source folder "~/catkin_ws/src"

Open a terminal :

    $ cd ~/catkin_ws
    $ catkin_make
    $ source ./devel/setup.bash


## Run the program

   Open a terminal :

   	$ roslaunch zed_wrapper zed_depth.launch

**WARNING : to get the depth in meters (it's a requirement for ROS), the baseline has to be set manually to 0.12 using ZED Settings App**

## Launch file parameters

Parameter       |           Description           |              Value               
------------------------|---------------------------------|---------------------------------
 computeDepth          | Toggle depth computation.       | '0': depth not computed, Left+Right images published 
                       |                                 | '1': depth computed, Left+Depth images published 
 svo_file              | SVO filename                    | path to an SVO file              
 resolution            | ZED Camera resolution           | '0': HD2K                        
                       |                                 | '1': HD1080                      
                       |                                 | '2': HD720                       
                       |                                 | '3': VGA                         
 quality               | Disparity Map quality           | '1': PERFORMANCE                 
                      |                                 | '2': QUALITY                     
 sensing_mode          | Depth sensing mode              | '0': FULL                        
                       |                                 | '1': RAW                         
frame_rate            | Rate at which images are published  | int                              
 left_topic            | Topic to which left images are published | string                           
 second_topic          | Topic to which depth or right images are published   | string                           
 left_cam_info_topic   | Topic to which left camera info are published | string                           
 second_cam_info_topic | Topic to which right or depth camera info are published  | string                           
 left_frame_id         | ID specified in the left image message header | string                           
 second_frame_id       | ID specified in the depth or right image message header   | string                           
