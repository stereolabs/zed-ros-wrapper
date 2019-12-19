LATEST CHANGES
==============
Differences with the latest release v2.8.x:
https://github.com/stereolabs/zed-ros-wrapper/releases/latest

- Add new parameter `self_calib` to enable/disable initial self calibration
- Add new parameter `imu_fusion` to enable/disable IMU fusion in visual odometry processing (only ZED-M)
- Updated timestamp in `camera_info` messages (Thx @abylikhsanov)
- Add new Color Enhancment parameter
- Add new Dynamic Parameters for camera settings:
    - Brightness
    - Contrast
    - Hue
    - Saturation
    - Sharpness
    - Auto White Balance
    - White Balance Temperature
- Support for the new ZED2 camera
- New sensors for ZED2:
    - Barometer
    - Magnetometer
    - Camera Temperatures
- Add CMOS temperatures to Diagnostic (only ZED2)
- Add SVO playing status to Diagnostic
- Remove max camera range from dynamic paramaters
- Add service to start/stop spatial mapping
- Add support for Object Detection (only ZED2)
    - Services to start/stop Object Detection
- Moved the `zed_display_rviz` package to a new separated repository: 
- Moved samples and tutorials to a new seaprated repository:


