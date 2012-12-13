cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++0x)

orocos_library(ASCLoader src/ASCLoader.cpp)

orocos_generate_package()
