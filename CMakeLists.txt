cmake_minimum_required(VERSION 2.8.7)

project(zed_wrapper)

###############################################################################
## Options
option( DEBUG_ACTIVE "Enable Debug build" ON )

if(DEBUG_ACTIVE)
    MESSAGE( "Debug compilation active" )
    set(ROS_BUILD_TYPE Debug)
    set(CMAKE_BUILD_TYPE Debug)
else()
    MESSAGE( "Release compilation active" )
    set(ROS_BUILD_TYPE Release)
    set(CMAKE_BUILD_TYPE Release)
endif()
###############################################################################

IF(WIN32) # Windows
SET(ZED_INCLUDE_DIRS $ENV{ZED_INCLUDE_DIRS})
    if (CMAKE_CL_64) # 64 bits
        SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_64})
    else(CMAKE_CL_64) # 32 bits
        SET(ZED_LIBRARIES $ENV{ZED_LIBRARIES_32})
    endif(CMAKE_CL_64)
SET(ZED_LIBRARY_DIR $ENV{ZED_LIBRARY_DIR})
SET(OPENCV_DIR $ENV{OPENCV_DIR})
ELSE() # Linux
find_package(ZED 0.9 REQUIRED)
ENDIF(WIN32)

find_package(CUDA 6.5 REQUIRED)
find_package(OpenCV 2.4 COMPONENTS core highgui imgproc REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  rosconsole
  sensor_msgs
  cv_bridge
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    opencv2
    cv_bridge
    image_transport
)

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS} 
	${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

add_definitions(-std=c++11)# -m64) #-Wall)

add_executable(
  zed_wrapper_node
  src/zed_wrapper_node.cpp
)

target_link_libraries(
        zed_wrapper_node
        ${catkin_LIBRARIES}
        ${ZED_LIBRARIES}
	${CUDA_LIBRARIES} ${CUDA_nppi_LIBRARY} ${CUDA_npps_LIBRARY}
        ${OpenCV_LIBS}  
    )
###############################################################################

