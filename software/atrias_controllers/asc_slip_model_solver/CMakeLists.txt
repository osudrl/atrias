#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

project(asc_slip_model_solver)

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

rosbuild_find_ros_package(atrias)
if(DEFINED atrias_PACKAGE_PATH)
	include(${atrias_PACKAGE_PATH}/atrias.cmake)
else(DEFINED atrias_PACKAGE_PATH)
	message(ERROR "Could not find package atrias. I'm not going to build anything!")
endif(DEFINED atrias_PACKAGE_PATH)

# Find RTT libraries and build Orocos Component.
if(ATRIAS_BUILD_CONTROLLERS)
	#
	# Components, types and plugins.
	#
	# The CMake 'target' names are identical to the first argument of the
	# macros below, except for orocos_typegen_headers, where the target is fully
	# controlled by generated code of 'typegen'.
	#

	#
	# Building a Service:
	#
	# Creates a plugin library libasc_slip_model_solver-service-<target>.so
	# and installs in the directory lib/orocos/asc_slip_model_solver/plugins/
	#
	orocos_service(asc_slip_model_solver-service src/asc_slip_model_solver-service.cpp) # ...only one service per library !
	#
	# You may add multiple orocos_service statements.


	#
	# Generates and installs our package. Must be the last statement such
	# that it can pick up all above settings.
	#
	orocos_generate_package()
endif(ATRIAS_BUILD_CONTROLLERS)

