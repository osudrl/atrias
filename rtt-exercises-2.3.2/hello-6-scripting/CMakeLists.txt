#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
# The CMake 'target' names are identical to the first argument of the
# macro, except for orocos_typekit, where the target is fully
# controlled by generated code of 'typegen'.
#
project(hello-6-scripting)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  rosbuild_find_ros_package( ocl )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
  set( OCL_HINTS HINTS ${ocl_PACKAGE_PATH}/../install )
endif()

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
# NOTE: we add rtt-scripting as a plugin to look for:
find_package(Orocos-RTT REQUIRED rtt-scripting ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

# Creates an orocos executable HelloWorld
orocos_use_package(ocl-taskbrowser)
orocos_executable(HelloWorld HelloWorld.cpp) # ...you may add multiple source files

# Link with the rtt-scripting plugin, since we include a header of it (rtt/scripting/Scripting.hpp):
target_link_libraries(HelloWorld ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY})

#
# Makes this an installable/usable package for other packages.
#
orocos_generate_package()
