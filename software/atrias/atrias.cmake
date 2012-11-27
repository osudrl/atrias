
# Specify build targets here.
set(ATRIAS_BUILD_GUI 1)
set(ATRIAS_BUILD_CONTROLLERS 0)


### DO NOT EDIT BELOW THIS LINE ###

# Check that GTK libraries exist. If not, do not try to build the GUI.
if(ATRIAS_BUILD_GUI)
	find_package(GTK2 2.4 COMPONENTS gtk gtkmm REQUIRED)
	if(NOT GTK2_FOUND)
		set(ATRIAS_BUILD_GUI 0)
		message(ERROR "Could not find GTK libraries. Not building GUI elements.")
	endif(NOT GTK2_FOUND)
endif(ATRIAS_BUILD_GUI)

# Check that Orocos RTT exists. If not, do not try to build the controller
# components.
if(ATRIAS_BUILD_CONTROLLERS)
	rosbuild_find_ros_package(rtt)
	if(NOT DEFINED rtt_PACKAGE_PATH)
		set(ATRIAS_BUILD_CONTROLLERS 0)
		message(ERROR "Could not find rtt. Not building controller components.")
	endif(NOT DEFINED rtt_PACKAGE_PATH)
endif(ATRIAS_BUILD_CONTROLLERS)

