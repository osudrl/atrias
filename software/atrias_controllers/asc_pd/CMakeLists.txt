cmake_minimum_required(VERSION 2.6.3)
project(asc_pd)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
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

# Find RTT libraries and build Orocos Component.
if(ATRIAS_BUILD_CONTROLLERS)
	# Build old-style controller for backwards compatibility
	#orocos_component(pd_controller src/controller_component.cpp)

	# ... and the new-style controller
	orocos_library(ASCPD src/ASCPD.cpp)
	target_link_libraries(ASCPD ControlLib-${OROCOS_TARGET})

	# Build typekits
	ros_generate_rtt_typekit(asc_pd)
endif(ATRIAS_BUILD_CONTROLLERS)
