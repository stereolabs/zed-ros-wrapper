# Stereolabs ZED Camera - Alvar AR tag example

This package shows how to use the ZED Wrapper with [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)

## Run the program

To launch the example, open a terminal and launch:

    $ roslaunch zed_ar_track_alvar_example zed_ar_track_alvar.launch

The launch file automatically starts the ZED node, the ar_track_alvar node and RVIZ with a preconfigured view:

![Example of indoor 3D mapping](images/ar_track_alvar.png)

The `Left Image` view shows the RGB stream from the left camera of the ZED.
The `Left Camera` view shows the "world" as seen by the left camera. The `TF` and the `Tags` are selected to be shown in 3D projection
The 3D view shows the ZED camera localized in the space and the position of the `Tags` as coloured markers.

*Note* It is important to subscribe the `/zed/pose` topic to be able to localize the camera in the space.

*Note* the tags used in the example are available following this [link](http://wiki.ros.org/ar_track_alvar?action=AttachFile&do=get&target=markers0to8.png)

## The launch file explained

To correctly use the ZED wrapper with the `ar_track_alvar` node we need to match the following ar_track_alvar parameters:

- `cam_image_topic` -> topic of the color image used to detect the tags
- `cam_info_topic` -> topic of camera parameters used to create the association of each color pixel to the relative 3D point
- `camera_frame` -> the name of the TF frame associated to the camera position

The values associated to the above parameters are the following:

```
    <arg name="cam_image_topic"         default="rgb/image_rect_color" />
    <arg name="cam_info_topic"       	default="rgb/camera_info" />
    <arg name="camera_frame"            default="zed_left_camera_frame" />
```

The corresponding parameters of the ZED node are the following:

- `cam_image_topic` -> `rgb_topic`
- `cam_info_topic` -> `rgb_info_topic`
- `camera_frame` -> `left_camera_frame`

The corresponding parameters of the `rtabmap_ros` node are the following:

- `cam_image_topic` -> `camera_image`
- `cam_info_topic` -> `camera_info`
- `camera_frame` -> `output_frame`

It is also important to set the correct `marker_size` in millimeters. Better if measured after printing them:

    <arg name="marker_size" default="5.0" />





