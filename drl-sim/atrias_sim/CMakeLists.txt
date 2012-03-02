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
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})


# TODO: This shouldn't be necessary once controllers are modular.
rosbuild_find_ros_package(atrias_controllers)
include_directories(${atrias_controllers_PACKAGE_PATH}/include)
set(CONTROLLERS ${atrias_controllers_PACKAGE_PATH}/src/no_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/motor_torque_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/motor_position_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/leg_torque_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/leg_position_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/leg_angle_sin_wave.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/leg_force_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/raibert_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/hubicki_controller.cpp
                ${atrias_controllers_PACKAGE_PATH}/src/test_controller.cpp)

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

    # XXX These need to be fixed and re-included.
    rosbuild_add_library(all_in_one_controller_wrapper src/all_in_one_controller_wrapper.cpp src/control_switcher_state_machine.cpp ${CONTROLLERS})
        target_link_libraries(all_in_one_controller_wrapper ${ECAT_LIBRARY})
    rosbuild_link_boost(all_in_one_controller_wrapper thread)

    #rosbuild_add_library(ecat_interface src/ecat_interface.cpp)
    #rosbuild_link_boost(ecat_interface thread)
else(EXISTS "${GAZEBO_INSTALLED}")
    message("Gazebo not detected, so not going to build the simulation.")
endif(EXISTS "${GAZEBO_INSTALLED}")

