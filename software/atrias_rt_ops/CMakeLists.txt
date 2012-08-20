#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

project(atrias_rt_ops)

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



#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} additional_libraries)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example src/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

#rosbuild_add_executable(fraggleschmidt test.cpp src/ECatComm.cpp src/ECatSlavesManager.cpp src/TestMedulla.cpp src/Communicator.cpp src/MedullaManager.cpp src/LegMedulla.cpp)

#target_link_libraries(fraggleschmidt rt)


#set(CONTROLLERS src/no_controller.cpp src/motor_torque_controller.cpp src/motor_position_controller.cpp src/leg_torque_controller.cpp src/leg_position_controller.cpp src/leg_angle_sin_wave.cpp src/raibert_controller.cpp src/test_controller.cpp src/leg_force_controller.cpp)

#rosbuild_add_library(${PROJECT_NAME} src/controller_wrapper_states.cpp src/control_switcher_state_machine.cpp ${CONTROLLERS})
#
#rosbuild_add_executable(uspace_controller src/uspace_controller_wrapper.cpp)
#target_link_libraries(uspace_controller ${PROJECT_NAME})
#target_link_libraries(uspace_controller ${ECAT_LIBRARY})


rosbuild_find_ros_package( rtt )
#rosbuild_find_ros_package( ocl )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )
#set( OCL_HINTS HINTS ${ocl_PACKAGE_PATH}/../install )

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#orocos_use_package(ocl-taskbrowser)

#rosbuild_add_library(soemtest SOEMTest.cpp)
#orocos_library(soemtest SOEMTest.cpp)
#rosbuild_add_executable(test2 test2.cpp)
#orocos_executable(test2 test2.cpp)
#target_link_libraries(test2 soemtest)

include_directories(../../robot_definitions/)
orocos_component(RTOps src/RTOps.cpp src/NoopConnector.cpp src/ECatComm.cpp src/BoomMedulla.cpp src/ECatSlavesManager.cpp src/Communicator.cpp src/MedullaManager.cpp src/Medulla.cpp src/LegMedulla.cpp)
orocos_component(DeployerAffinitySetter src/DeployerAffinitySetter.cpp)
target_link_libraries(RTOps rt)
#include_directories(/opt/etherlab/include)

orocos_generate_package()

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libatrias_ecat_master-<target>.so
# and installs in the directory lib/orocos/atrias_ecat_master/
#
#orocos_component(atrias_ecat_master src/atrias_ecat_master-component.cpp src/uspace_controller_wrapper.cpp src/controller_wrapper_states.cpp src/control_switcher_state_machine.cpp ${CONTROLLERS} ${ECAT_LIBRARY}) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/atrias_ecat_master/ directory
#
#orocos_install_headers( atrias_ecat_master-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libatrias_ecat_master-plugin-<target>.so
# and installs in the directory lib/orocos/atrias_ecat_master/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(DeployerAffinitySetter-plugin src/DeployerAffinitySetter.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libatrias_ecat_master-service-<target>.so
# and installs in the directory lib/orocos/atrias_ecat_master/plugins/
#
#orocos_service(OrocosAffinitySetter-service src/OrocosAffinitySetter.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit (recommended):
#
# Creates a typekit library libatrias_ecat_master-types-<target>.so
# and installs in the directory lib/orocos/atrias_ecat_master/types/
#
#orocos_typegen_headers(atrias_ecat_master-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support src/uspace_controller_wrapper.cpp src/controller_wrapper_states.cpp src/control_switcher_state_machine.cpp ${CONTROLLERS} ${ECAT_LIBRARY}) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
#orocos_generate_package()


# Generate RTT typekits. This is based on the CMakeLists.txt generated by
# create-rtt-msgs from the package rtt_rosnode.
rosbuild_include(rtt_rosnode GenerateRTTtypekit)
ros_generate_rtt_typekit(atrias_rt_ops)
