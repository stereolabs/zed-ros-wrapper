cmake_minimum_required(VERSION 2.8.7)

project(zed_interfaces)

#Fix for QtCretor
#list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/")

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  sensor_msgs
  #actionlib_msgs
  message_generation
)

add_message_files( 
    DIRECTORY msg 
    FILES
    ObjectStamped.msg
    Objects.msg
    RGBDSensors.msg
  )

#add_action_files(DIRECTORY action FILES act.action)

add_service_files( 
    DIRECTORY srv 
    FILES
    set_pose.srv
    reset_odometry.srv
    reset_tracking.srv
    start_svo_recording.srv
    stop_svo_recording.srv
    start_remote_stream.srv
    stop_remote_stream.srv
    set_led_status.srv
    toggle_led.srv
    start_3d_mapping.srv
    stop_3d_mapping.srv
    start_object_detection.srv
    stop_object_detection.srv
  )

generate_messages(
    DEPENDENCIES
    #actionlib_msgs
    std_msgs
    geometry_msgs
    sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS 
    message_generation 
    std_msgs 
    geometry_msgs 
    sensor_msgs
    #actionlib_msgs
)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})


