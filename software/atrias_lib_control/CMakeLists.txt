cmake_minimum_required(VERSION 2.6.3)
project(atrias_lib_control)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# C++11 support
add_definitions(-std=c++0x)

orocos_library(LibControl src/AtriasController.cpp src/ATC.cpp)

orocos_generate_package()
