master branch
-------------
- Add new parameter `self_calib` to enable/disable initial self calibration
- Add new parameter `imu_fusion` to enable/disable IMU fusion in visual odometry processing (only ZED-M)

==========

v2.8.x
------
- Updated API for compatibility with 2.8 release.
- Moved parameters from launch files to YAML files. This makes them clearer and unifies parameters between different launch files.
- Changed default prefix of the topics adding node name: /<namespace>/<node_name>/<topic> (default /zed/zed_node/<topic>)
- Added minimum depth parameter (min_depth).
- Added base_link to the TF tree to improve user experience when adding ZED cameras to a standard ROS robot configuration. Pose and Odometry are now referred to base_link instead of zed_camera_center.
- Added a separated launch file for ZED Mini (zedm.launch and display_zedm.launch).
- Added a service to set the ON/OFF status of the blue LED (firmware > 1523).
- Added a service to toggle the status of the blue led (firmware > 1523).
- Added a parameter to choose the default SVO compression mode for SVO recording.
- Added 2D mode for stable navigation on planar surfaces.
- Added option to set a fixed pose/odometry covariance or use the dynamic matrices calculated by the SDK.
- Added SVO recording services.
- Added zero-copy to Pointcloud publishing (thanks @RhysMcK).
- Added dynamic parameter to change the frequency of point cloud publishing. Now the point cloud can have a frequency different from the grab frequency (always less than or equal to the grab frequency).
- Added services to start/stop network streaming, which allows remote machines to acquire and process images using a ROS node or a "not ROS" application.
- Added stereo side-by-side image topic for raw and rectified images.
- Removed initial warning about TF not being available.
- Improved console logging for a better vision of the node configuration.
- Fixed additional minor issues.

==========

v2.7.x
------
- Added Floor Alignment feature (with SDK > v2.6).
- Added Covariance information to the odometry message (with SDK > v2.6).
- Added the new topic "pose_with_covariance" (with SDK > v2.6).
- Added Odometry and Pose path messages.
- Added parameter to disable the publishing of the map frame.
- Added parameter to disable the publishing of the covariance in odometry and pose.
- Updated the structure of the plugins in RVIZ display.
- New tutorials about Video, Depth and Pose subscription in C++.
- Moved zed_nodelet_example and zed_rtabmap_example in the new "examples" folder.
- New documentation with new tutorials about Video, Depth and Positional Tracking modules.
- Fixed reconnection issue. Thanks to @RhysMcK.
- Fixed Pointcloud structure issue. Thanks to @ChristofDubs .
- Removed OpenCV dependency.

==========

v2.5.x - v2.6.x
---------------
- The wrapper is now divided in sub-packages
- RViz display launch files for ZED and ZED mini in the new zed_display_rviz sub-package
- Added RTABMap example in the new zed_rtabmap_example sub-package
- Added Nodelet intraprocess communication example in the new zed_nodelet_example sub-package
- Added parameter to enable Camera Flip. Thanks to @frk2
- Improved Pointcloud2 topic publishing.
- Bug fixes

==========

v2.3.x - v2.4.x
---------------
- For the ZED SDK 2.3 and 2.4

==========

v2.2.x
------
- For the ZED SDK 2.2.x

==========

v2.1.x
------
- For the ZED SDK 2.1.x

==========

v1.2.0
------
- For the ZED SDK 1.2.0

==========

v1.0.0
------
- For the ZED SDK 1.0.0

==========

v0.9.X
------
- First release of the wrapper for the ZED SDK 0.9.X


