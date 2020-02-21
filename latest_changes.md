LATEST CHANGES
==============

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





