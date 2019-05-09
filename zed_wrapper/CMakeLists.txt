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

find_package(CUDA)
checkPackage("CUDA" "CUDA not found, install it from:\n https://developer.nvidia.com/cuda-downloads")

find_package(OpenMP)
checkPackage("OpenMP" "OpenMP not found, please install it to improve performances: 'sudo apt install libomp-dev'")
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin COMPONENTS
  image_transport
  roscpp
  rosconsole
  sensor_msgs
  stereo_msgs
  dynamic_reconfigure
  tf2_ros
  nodelet
  tf2_geometry_msgs
  message_generation
  diagnostic_updater
  roslint
)

checkPackage("image_transport" "")
checkPackage("roscpp" "")
checkPackage("rosconsole" "")
checkPackage("sensor_msgs" "")
checkPackage("stereo_msgs" "")
checkPackage("dynamic_reconfigure" "")
checkPackage("tf2_ros" "")
checkPackage("nodelet" "")
checkPackage("tf2_geometry_msgs" "")
checkPackage("message_generation" "")

add_service_files( FILES
    set_pose.srv
    reset_odometry.srv
    reset_tracking.srv
    start_svo_recording.srv
    stop_svo_recording.srv
    start_remote_stream.srv
    stop_remote_stream.srv
    set_led_status.srv
    toggle_led.srv
  )

generate_messages()

generate_dynamic_reconfigure_options(
  cfg/Zed.cfg
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    stereo_msgs
    image_transport
    dynamic_reconfigure
    tf2_ros
    tf2_geometry_msgs
    message_runtime
)

###############################################################################
# SOURCES

set(TOOLS_SRC
    ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/src/sl_tools.cpp)
set(NODE_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/zed_wrapper_node.cpp)
set(NODELET_SRC ${CMAKE_CURRENT_SOURCE_DIR}/src/nodelet/src/zed_wrapper_nodelet.cpp)

###############################################################################

###############################################################################
# INCLUDES

# Specify locations of header files.
include_directories(
        ${catkin_INCLUDE_DIRS}
        ${CUDA_INCLUDE_DIRS}
        ${ZED_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tools/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/nodelet/include
)

link_directories(${ZED_LIBRARY_DIR})
link_directories(${CUDA_LIBRARY_DIRS})

###############################################################################

###############################################################################
# EXECUTABLE

if(NOT DEFINED CUDA_NPP_LIBRARIES_ZED)
    #To deal with cuda 9 nppi libs and previous versions of ZED SDK
    set(CUDA_NPP_LIBRARIES_ZED ${CUDA_npp_LIBRARY} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY})   
endif()


add_definitions(-std=c++11)
set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  ${ZED_LIBRARIES}
  ${CUDA_LIBRARIES} ${CUDA_NPP_LIBRARIES_ZED}
  )

add_library(ZEDWrapper ${TOOLS_SRC} ${NODELET_SRC})
target_link_libraries(ZEDWrapper ${LINK_LIBRARIES})
add_dependencies(ZEDWrapper ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(zed_wrapper_node ${NODE_SRC})
target_link_libraries(zed_wrapper_node ZEDWrapper ${LINK_LIBRARIES})
add_dependencies(zed_wrapper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

###############################################################################

#Add all files in subdirectories of the project in
# a dummy_target so qtcreator have access to all files
FILE(GLOB_RECURSE all_files ${CMAKE_SOURCE_DIR}/*)
add_custom_target(all_files_${PROJECT_NAME} SOURCES ${all_files})

roslint_cpp()


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
  params
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
