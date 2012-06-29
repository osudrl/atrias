#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(atrias_controller_wrapper)

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

rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )

# Set the CMAKE_PREFIX_PATH in case you're not using Orocos through ROS
# for helping these find commands find RTT.
find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})

# Defines the orocos_* cmake macros. See that file for additional
# documentation.
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

#
# Components, types and plugins.
#
# The CMake 'target' names are identical to the first argument of the
# macros below, except for orocos_typegen_headers, where the target is fully
# controlled by generated code of 'typegen'.
#

# Creates a component library libatrias_controller_wrapper-<target>.so
# and installs in the directory lib/orocos/atrias_controller_wrapper/
#
orocos_component(atrias_controller_wrapper atrias_controller_wrapper.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

orocos_library(atrias_controller_wrapper_library src/controller_wrapper_states.cpp ${CONTROLLERS})

target_link_libraries(atrias_controller_wrapper atrias_controller_wrapper_library)

orocos_generate_package()

#
# Additional headers:
#
# Installs in the include/orocos/atrias_controller_wrapper/ directory
#
#orocos_install_headers( atrias_controller_wrapper-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Plugin
#
# Creates a plugin library libatrias_controller_wrapper-plugin-<target>.so
# and installs in the directory lib/orocos/atrias_controller_wrapper/plugins/
#
# Be aware that a plugin may only have the loadRTTPlugin() function once defined in a .cpp file.
# This function is defined by the plugin and service CPP macros.
#
#orocos_plugin(atrias_controller_wrapper-plugin atrias_controller_wrapper-plugin.cpp) # ...only one plugin function per library !
#
# You may add multiple orocos_plugin statements.


#
# Building a Service:
#
# Creates a plugin library libatrias_controller_wrapper-service-<target>.so
# and installs in the directory lib/orocos/atrias_controller_wrapper/plugins/
#
#orocos_service(atrias_controller_wrapper-service atrias_controller_wrapper-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Building a typekit (recommended):
#
# Creates a typekit library libatrias_controller_wrapper-types-<target>.so
# and installs in the directory lib/orocos/atrias_controller_wrapper/types/
#
#orocos_typegen_headers(atrias_controller_wrapper-types.hpp) # ...you may add multiple header files
#
# You may only have *ONE* orocos_typegen_headers statement !


#
# Building a normal library (optional):
#
# Creates a library libsupport-<target>.so and installs it in
# lib/
#
#orocos_library(support support.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_library statements.


#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
#orocos_generate_package()
