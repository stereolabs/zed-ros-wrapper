LATEST CHANGES
==============

XACRO (2020-01-31)
------------------
- Added xacro support
- Fixed auto white balance at node start (thanks to @kjaget)
- Removed `fixed_covariance` and `fixed_cov_value` parameters (not required anymore)
- Removed `sens_pub_rate` parameter
- Removed `confidence_image`


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





