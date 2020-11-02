LATEST CHANGES
==============

RGB/Depth sync fix #629 (2020-11-02)
-------------------------------
- Fixed sync issue between RGB and Depth data (Thx @dennisVi)
- Fixed issues with SVO and sensors data (Thx @dennisVi)

ASYNC Object Detection (2020-09-18)
-----------------------------------
- Object Detection now runs asynchronously respect to data grabbing and Object Detected data are published only when available not affecting the frequency of the publishing of the other data types
- Depth OpenNI topic name changed from `depth/depth_raw_registered` to `depth/depth_registered`

IMU timestamp fix (2020-08-25)
------------------------------
- Added new parameter `sensors/publish_imu_tf` to enable/disable IMU TF broadcasting
- Fixed duplicated IMU timestamp issue (see ticket #577)
- Fixed problem with IMU TF in Rviz: `odom` and `zed_camera_center` TFs are now published at the same frequency of the IMU TF, if available)
- IMU TF is now published once as static TF even if the IMU topic is not subscribed

Timestamp fix (2020-06-03)
--------------------------
- Fix timestamp update coherency due to parallel threads. Thanks to @matlabbe

IMU fix (2020-05-24)
--------------------
- Fix issue with IMU frame link when `publish_tf` and `publish_map_tf` are disabled

New package: zed_nodelets (2020-03-20)
---------------------------------------
- Added the new `zed_interfaces/RGBDSensors` custom topic that contains RGB, Depth, IMU and Magnetometer synchronized topics
- Added a new package `zed_nodelets` that contains the main `zed_nodelets/ZEDWrapperNodelet` and new nodelets
- Added a new nodelet `zed_nodelets/RgbdSensorsSyncNodelet` that subscribes to RGB, Depth, IMU and Magnetometer topics and republish them in a single synchronized message
- Added a new nodelet `zed_nodelets/RgbdSensorsDemuxNodelet` that subscribes to RGBDSensors and republish RGB, Depth, IMU and Magnetometer as single topics
- Renamed `zed_interfaces/objects` to `zed_interfaces/Objects`
- Renamed `zed_interfaces/object_stamped` to `zed_interfaces/ObjectStamped`
- Reorganized the `zed_wrapper/launch` folder adding the `include` folder
- New online documentation to explain in details the new `zed_nodelets` package: https://www.stereolabs.com/docs/ros/zed_nodelets/

v3.1
-----
- Added new package `zed_interfaces` with isolated declarations of custom messages, services and actions
- Removed not used `world_frame` parameter
- Removed the`publish_pose_covariance` parameter, now covariance for pose and odometry is always published
- Removed `_m` from parameters `mapping/resolution_m` and `mapping/max_mapping_range_m`
- Renamed the parameter `depth_resample_factor` to `depth_downsample_factor`
- Renamed the parameter `img_resample_factor` to `img_downsample_factor`
- Renamed the parameter `odometry_db` to `area_memory_db_path`
- Renamed the parameter `frame_rate` to `grab_frame_rate`
- Added new dynamic parameter `pub_frame_rate` to reduce Video and Depth publishing frequency respect to grabbing frame rate [`grab_frame_rate`]
- Added new dynamic parameter `gamma` for Gamma Control
- Added new dynamic parameter `depth_texture_conf` to filter depth according to textureness information
- Added new TF frames for all the sensors available on ZED2
- Added publishers for gray images 
- Added publisher for Camera to IMU transform: `/<camera_name>/<node_name>/camera_imu_transform`
- Default value for `depth_confidence` changed from 100 to 50
- Added `base_frame` as launch parameter to propagate the value of the parameter in the Xacro description


Bug fix (2020-03-06)
--------------------
- Fix default value for dynamic parameters not set from `common.yaml`

XACRO and more (2020-01-31)
---------------------------
- Added xacro support for parametric URDF 
- Removed redundant URDFs and added a single parametric URDF based on xacro
- Fixed auto white balance at node start (thanks to @kjaget)
- Removed `fixed_covariance` and `fixed_cov_value` parameters (not required anymore)
- Removed `sens_pub_rate` parameter
- Removed `confidence_image` message
- Removed `color_enhancement` parameter, always ON by default
- Mapping does not use presets for resolution, but a float value in range [0.01,0.2]
- Added new parameter `max_mapping_range_m` for mapping depth range (set to `-1` for auto calculation)
- Moved "multi-camera" launch file in [`zed-ros-examples`](https://github.com/stereolabs/zed-ros-examples/tree/master/examples/zed_multicamera_example) 
- Added current GPU ID to Diagnostic information
- The `confidence` dynamic parameter is now called `depth_confidence`
- Removed dynamic parametes `map_resize_factor`
- Added new parameter `video/img_resample_factor`
- Added new parameter `depth/map_resample_factor`
- Updated the names for the parameters of the Object Detection module [only ZED2]

SDK v3.0 (2020-01-27)
---------------------
- Added a new repository [`zed-ros-examples`](https://github.com/stereolabs/zed-ros-examples) to keep separated the main ZED Wrapper node from Examples and Tutorials. A clean robot installation is now allowed
- ZED 2 support
- Color enhancement support
- Max range is not a dynamic parameter anymore
- Camera temperature added to diagnostic (only ZED2)
- New service to start/stop mapping
- Support for Object Detection (only ZED2)
- Advanced support for on-board sensors (only ZED-M and ZED2)
- New tutorials, see [`zed-ros-examples`](https://github.com/stereolabs/zed-ros-examples)





