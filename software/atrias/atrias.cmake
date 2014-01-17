# Author: Soo-Hyun Yoo

# Specify build targets here.
set(ATRIAS_BUILD_GUI 1)
set(ATRIAS_BUILD_CONTROLLERS 0)


### DO NOT EDIT BELOW THIS LINE ###

# Include robot_definitions by guessing (relative to the directory of the
# project including this CMake file. This is shameful. There has to be a better
# way besides guessing.
include_directories(../../robot_definitions)
include_directories(../../../robot_definitions)

# Check that GTK libraries exist. If not, do not try to build the GUI.
if(ATRIAS_BUILD_GUI)
	find_package(GTK2 2.4 COMPONENTS gtk gtkmm REQUIRED)
	if(GTK2_FOUND)
		include_directories(${GTK2_INCLUDE_DIRS})
		message(INFO "Building GUI components.")
	else(GTK2_FOUND)
		set(ATRIAS_BUILD_GUI 0)
		message(ERROR "Could not find GTK libraries. Not building GUI elements.")
	endif(GTK2_FOUND)
endif(ATRIAS_BUILD_GUI)

# Check that Orocos RTT exists. If not, do not try to build the controller
# components.
if(ATRIAS_BUILD_CONTROLLERS)
	rosbuild_find_ros_package(rtt)
	rosbuild_find_ros_package(rtt_rosnode)

	if(DEFINED rtt_PACKAGE_PATH AND DEFINED rtt_rosnode_PACKAGE_PATH)
		set(RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install)
		find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
		include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
		message(INFO "Building controller components.")

		# The following removes the need to depend on rtt_rosnode in
		# manifest.xml.
		include(${rtt_rosnode_PACKAGE_PATH}/cmake/GenerateRTTtypekit.cmake)
		include_directories(${rtt_rosnode_PACKAGE_PATH}/src)
	else(DEFINED rtt_PACKAGE_PATH AND DEFINED rtt_rosnode_PACKAGE_PATH)
		set(ATRIAS_BUILD_CONTROLLERS 0)
		message(ERROR "Could not find rtt and/or rtt_rosnode. Not building controller components.")
	endif(DEFINED rtt_PACKAGE_PATH AND DEFINED rtt_rosnode_PACKAGE_PATH)
endif(ATRIAS_BUILD_CONTROLLERS)

