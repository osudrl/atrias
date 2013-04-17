#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

project(atc_eq_point)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

rosbuild_find_ros_package(atrias)
if(DEFINED atrias_PACKAGE_PATH)
	include(${atrias_PACKAGE_PATH}/atrias.cmake)
else(DEFINED atrias_PACKAGE_PATH)
	message(ERROR "Could not find package atrias. I'm not going to build anything!")
endif(DEFINED atrias_PACKAGE_PATH)

# Find GTK libraries and build GUI library.
if(ATRIAS_BUILD_GUI)
    rosbuild_add_library(controller_gui SHARED src/controller_gui.cpp)
    target_link_libraries(controller_gui ${GTK2_LIBRARIES})
endif(ATRIAS_BUILD_GUI)

# Find RTT libraries and build Orocos Component.
if(ATRIAS_BUILD_CONTROLLERS)
	orocos_component(ATCEqPoint src/ATCEqPoint.cpp)

	ros_generate_rtt_typekit(atc_eq_point)
endif(ATRIAS_BUILD_CONTROLLERS)

