cmake_minimum_required(VERSION 2.8.7)

project(zed_wrapper)

#Fix for QtCretor
#list(APPEND CMAKE_PREFIX_PATH "/opt/ros/$ENV{ROS_DISTRO}/")

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosconsole
  nodelet
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    zed_nodelets
)

###############################################################################
# SOURCES

set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_wrapper_node.cpp)
###############################################################################

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
)

add_executable(zed_wrapper_node ${NODE_SRC})
target_link_libraries(zed_wrapper_node ${LINK_LIBRARIES})
add_dependencies(zed_wrapper_node ${catkin_EXPORTED_TARGETS})


###############################################################################
#Add all files in subdirectories of the project in

# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

###############################################################################
# INSTALL

install(TARGETS
  zed_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY
  launch
  urdf
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
