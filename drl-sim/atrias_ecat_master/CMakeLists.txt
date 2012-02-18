#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

project(atrias_ecat_master)

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


#set(CONTROLLERS src/no_controller.cpp src/motor_torque_controller.cpp src/motor_position_controller.cpp src/leg_torque_controller.cpp src/leg_position_controller.cpp src/leg_angle_sin_wave.cpp src/raibert_controller.cpp src/test_controller.cpp src/leg_force_controller.cpp)

include_directories(/opt/etherlab/include)

find_library(ECAT_LIBRARY ethercat PATH /opt/etherlab/lib)
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


orocos_component(atrias_ecat_master atrias_ecat_master.cpp)
#orocos_executable(atrias_ecat_master atrias_ecat_master.cpp)

#orocos_install_headers(atrias_ecat_master-component.hpp)

#orocos_plugin(atrias_ecat_master-plugin atrias_ecat_master-plugin.cpp)

#orocos_service(atrias_ecat_master-service atrias_ecat_master-service.cpp)

#orocos_typegen_headers(atrias_ecat_master-types.hpp)


orocos_library(atrias_ecat_master_library src/uspace_controller_wrapper.cpp)

target_link_libraries(atrias_ecat_master_library ${ECAT_LIBRARY})
target_link_libraries(atrias_ecat_master atrias_ecat_master_library)


# Separate libraries to make debugging easier (without knowing how to use gdb).

#rosbuild_add_library(uspace_controller_wrapper src/uspace_controller_wrapper.cpp)
#rosbuild_add_library(controller_wrapper_states src/controller_wrapper_states.cpp)
#rosbuild_add_library(control_switcher_state_machine src/control_switcher_state_machine.cpp)
#rosbuild_add_library(controllers ${CONTROLLERS})

#target_link_libraries(atrias_ecat_master ${ECAT_LIBRARY})
#target_link_libraries(atrias_ecat_master uspace_controller_wrapper)
#target_link_libraries(atrias_ecat_master controller_wrapper_states)
#target_link_libraries(atrias_ecat_master control_switcher_state_machine)
#target_link_libraries(atrias_ecat_master controllers)

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
#orocos_plugin(atrias_ecat_master-plugin atrias_ecat_master-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libatrias_ecat_master-service-<target>.so
# and installs in the directory lib/orocos/atrias_ecat_master/plugins/
#
#orocos_service(atrias_ecat_master-service atrias_ecat_master-service.cpp) # ...only one service per library !
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

