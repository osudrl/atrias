# Devin Koepl

cmake_minimum_required(VERSION 2.4.6)
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
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_boost_directories()

# List of controllers.
set(CONTROLLERS src/no_controller.cpp src/motor_torque_controller.cpp src/motor_position_controller.cpp src/leg_torque_controller.cpp src/leg_position_controller.cpp src/leg_angle_sin_wave.cpp src/raibert_controller.cpp)

# Check to see whether RTAI is installed.
if(EXISTS /usr/realtime)
	# If it is, then build the RT wrappers.

	message("RTAI detected...")

	# Get RTAI compiler flags.
	execute_process(
		COMMAND rtai-config --lxrt-cflags
		OUTPUT_VARIABLE LXRT_CFLAGS
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	message("LXRT_CFLAGS: ${LXRT_CFLAGS}")
	execute_process(
		COMMAND rtai-config --lxrt-ldflags
		OUTPUT_VARIABLE LXRT_LDFLAGS
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	message("LXRT_LDFLAGS: ${LXRT_LDFLAGS}")
	# Remove the -W strict-prototypes flag, because it is not a valid flag for C++.
	string(REPLACE "-Wstrict-prototypes" "" LXRT_CFLAGS ${LXRT_CFLAGS})

	# Get ECAT compiler flags.
	set(ECAT_CFLAGS "-I /opt/etherlab/include -L /opt/etherlab/lib -l ethercat")
	message("ECAT FLAGS: ${ECAT_CFLAGS}")

	# Add RTAI and ECAT compiler flags.
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${LXRT_CFLAGS} ${ECAT_CFLAGS}")
	#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${LXRT_CFLAGS} ${ECAT_CFLAGS}")
	message("CFLAGS: ${CMAKE_CXX_FLAGS}")

	# Add RTAI libraries to the library search path.
	set(CMAKE_LIBRARY_PATH "${CMAKE_LIBRARY_PATH}:/usr/realtime/lib")

	# Define path for log files, and add it as a definition.
	execute_process(
		COMMAND rospack find atrias
		OUTPUT_VARIABLE LOG_FILE_PATH
		OUTPUT_STRIP_TRAILING_WHITESPACE
	)
	set(LOG_FILENAME ${LOG_FILE_PATH}/log_files/atrias_log.dat)
	add_definitions(-DLOG_FILENAME=${LOG_FILENAME})

	message("Log Filename: ${LOG_FILENAME}")

	#rosbuild_add_executable(wireless_kern_interface src/wireless_kern_interface.cpp src/data_logger.cpp)

	rosbuild_add_executable(rt_atrias_controller_wrapper src/rt_atrias_controller_wrapper.cpp src/controller_wrapper_states.cpp src/control_switcher_state_machine.cpp ${CONTROLLERS})
else(EXISTS /usr/realtime)
	message("RTAI not detected, skipping building the wireless kernel interface.")
endif(EXISTS /usr/realtime)

# Check to see if gazebo installed.
execute_process(
	COMMAND rospack find gazebo
	OUTPUT_VARIABLE GAZEBO_INSTALLED
	OUTPUT_STRIP_TRAILING_WHITESPACE
)

# If it is then build the controller wrappers.
if(EXISTS "${GAZEBO_INSTALLED}")
	# Build the controller wrappers.

	message("Building the simulator.")

	rosbuild_add_library(all_in_one_controller_wrapper src/all_in_one_controller_wrapper.cpp src/control_switcher_state_machine.cpp ${CONTROLLERS})
	rosbuild_link_boost(all_in_one_controller_wrapper thread)

	#rosbuild_add_library(ecat_interface src/ecat_interface.cpp)
	#rosbuild_link_boost(ecat_interface thread)
else(EXISTS "${GAZEBO_INSTALLED}")
	message("Gazebo not detected, so not going to build the simulation.")
endif(EXISTS "${GAZEBO_INSTALLED}")

#rosbuild_add_executable(tester src/tester.cpp src/data_logger.cpp)
