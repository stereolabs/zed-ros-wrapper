cmake_minimum_required(VERSION 3.5)

project(zed_nodelets)

# if CMAKE_BUILD_TYPE is not specified, take 'Release' as default
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_BUILD_TYPE)

function(checkPackage package customMessage)
    set(varName "${package}_FOUND")
    if (NOT "${${varName}}")
        string(REPLACE "_" "-" aptPackage ${package})
        if("${customMessage}" STREQUAL "")
            message(FATAL_ERROR "\n\n ${package} is missing, please try to install it with:\n sudo apt-get install ros-$(rosversion -d)-${aptPackage}\n\n")
        else()
            message(FATAL_ERROR "\n\n ${customMessage} \n\n")
        endif()
    endif()
endfunction(checkPackage)

find_package(ZED 3)
checkPackage("ZED" "ZED SDK v3.x not found, install it from:\n https://www.stereolabs.com/developers/")

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(CUDA)
checkPackage("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(OpenMP)
checkPackage("OpenMP" "OpenMP not found, please install it to improve performances: 'sudo apt install libomp-dev'")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
    roscpp
    image_transport
    rosconsole
    sensor_msgs
    stereo_msgs
    std_msgs
    message_filters
    tf2_ros
    nodelet
    tf2_geometry_msgs
    message_generation
    diagnostic_updater    
    dynamic_reconfigure
    zed_interfaces
)

generate_dynamic_reconfigure_options(
  cfg/Zed.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    stereo_msgs
    message_filters
    image_transport
    dynamic_reconfigure
    tf2_ros
    tf2_geometry_msgs
    message_runtime    
    zed_interfaces
    geometry_msgs
    visualization_msgs
)

###############################################################################
# SOURCES

set(TOOLS_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_tools.cpp)
set(ZED_NODELET_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_nodelet/src/zed_wrapper_nodelet.cpp)
set(RGBD_SENS_SYNC_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd_sensors_sync_nodelet/src/rgbd_sensor_sync.cpp)
set(RGBD_SENS_DEMUX_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd_sensors_demux_nodelet/src/rgbd_sensor_demux.cpp)

###############################################################################

###############################################################################
# INCLUDES

# Specify locations of header files.
set(INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${ZED_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_nodelet/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd_sensors_sync_nodelet/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/rgbd_sensors_demux_nodelet/include
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

###############################################################################

###############################################################################
# ZED WRAPPER NODELET

add_definitions(-std=c++11 -Wno-deprecated-declarations)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES}
  stdc++fs
)

add_library(ZEDNodelets
    ${TOOLS_SRC}
    ${ZED_NODELET_SRC}
    ${RGBD_SENS_SYNC_SRC}
    ${RGBD_SENS_DEMUX_SRC}
)
target_include_directories(ZEDNodelets PRIVATE ${INCLUDE_DIRS})
target_link_libraries(ZEDNodelets ${LINK_LIBRARIES})
add_dependencies(
    ZEDNodelets
    ${catkin_EXPORTED_TARGETS}
    ${PROJECT_NAME}_gencfg
)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

###############################################################################
# INSTALL

install(TARGETS
  ZEDNodelets
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

