cmake_minimum_required(VERSION 2.8.7)

project(zed_wrapper)

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

find_package(ZED 2.3)
checkPackage("ZED" "ZED SDK not found, install it from:\n https://www.stereolabs.com/developers/")

exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" ) # Jetson TX
    SET(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
endif()

find_package(OpenCV 3 COMPONENTS core highgui imgproc)
checkPackage("OPENCV_CORE" "OpenCV core not found, install it from the tutorial at:\n https://www.stereolabs.com/documentation/overview/getting-started/getting-started.html")
checkPackage("OPENCV_HIGHGUI" "OpenCV highgui not found, install it from the tutorial at:\n https://www.stereolabs.com/documentation/overview/getting-started/getting-started.html")
checkPackage("OPENCV_IMGPROC" "OpenCV imgproc not found, install it from the tutorial at:\n https://www.stereolabs.com/documentation/overview/getting-started/getting-started.html")

find_package(CUDA)
checkPackage("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(PCL)
checkPackage("PCL" "PCL not found, try to install it with:\n sudo apt-get install libpcl1 ros-$(rosversion -d)-pcl-ros")

find_package(catkin COMPONENTS
  image_transport
  roscpp
  rosconsole
  sensor_msgs
  dynamic_reconfigure
  tf2_ros
  pcl_conversions
  nodelet
  tf2_geometry_msgs
)

checkPackage("image_transport" "")
checkPackage("roscpp" "")
checkPackage("rosconsole" "")
checkPackage("sensor_msgs" "")
checkPackage("dynamic_reconfigure" "")
checkPackage("tf2_ros" "")
checkPackage("pcl_conversions" "")
checkPackage("nodelet" "")
checkPackage("tf2_geometry_msgs" "")

generate_dynamic_reconfigure_options(
  cfg/Zed.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    opencv
    image_transport
    dynamic_reconfigure
    tf2_ros
    tf2_geometry_msgs
    pcl_conversions
)
###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
	${catkin_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${ZED_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE


if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})   
endif()


add_definitions(-std=c++11)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## if using from pcl package (pcl/vtk bug)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES})

add_library(ZEDWrapper src/zed_wrapper_nodelet.cpp)
target_link_libraries(ZEDWrapper ${LINK_LIBRARIES})
add_dependencies(ZEDWrapper ${PROJECT_NAME}_gencfg)

add_executable(zed_wrapper_node src/zed_wrapper_node.cpp)
target_link_libraries(zed_wrapper_node ZEDWrapper ${LINK_LIBRARIES})
add_dependencies(zed_wrapper_node ${PROJECT_NAME}_gencfg)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE extra_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(dummy_${PROJECT_NAME} SOURCES ${extra_files})

###############################################################################
# INSTALL

install(TARGETS
  ZEDWrapper
  zed_wrapper_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(FILES
  nodelet_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
