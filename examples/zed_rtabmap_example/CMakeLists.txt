cmake_minimum_required(VERSION 2.8.7)

project(zed_rtabmap_example)

find_package(catkin REQUIRED)

catkin_package()

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INSTALL

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
