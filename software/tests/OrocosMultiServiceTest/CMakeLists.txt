#
# The find_package macro for Orocos-RTT works best with
# cmake >= 2.6.3
#
cmake_minimum_required(VERSION 2.6.3)

#
# This creates a standard cmake project. You may extend this file with
# any cmake macro you see fit.
#
project(OrocosServiceTest)

#
# Do setup in case of ros package, If ROS_ROOT is set, it is
# recommended to use RTT/OCL through the ros packages.
#
set (ROS_ROOT $ENV{ROS_ROOT} )
if (ROS_ROOT)
  include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
  rosbuild_init()
  rosbuild_find_ros_package( rtt )
  set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/install )
endif()

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

# Creates a component library libOrocosServiceTest-<target>.so
# and installs in the directory lib/orocos/OrocosServiceTest/
#
orocos_component(OrocosServiceTest OrocosServiceTest-component.hpp OrocosServiceTest-component.cpp) # ...you may add multiple source files
orocos_component(OrocosServiceTestAlt OrocosServiceTestAlt-component.hpp OrocosServiceTestAlt-component.cpp) # ...you may add multiple source files
#
# You may add multiple orocos_component statements.

#
# Additional headers:
#
# Installs in the include/orocos/OrocosServiceTest/ directory
#
orocos_install_headers( OrocosServiceTest-component.hpp ) # ...you may add multiple header files
#
# You may add multiple orocos_install_headers statements.

#
# Building a Service:
#
# Creates a plugin library libOrocosServiceTest-service-<target>.so
# and installs in the directory lib/orocos/OrocosServiceTest/plugins/
#
orocos_service(OrocosServiceTest-service OrocosServiceTest-service.cpp) # ...only one service per library !
#
# You may add multiple orocos_service statements.


#
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
orocos_generate_package()
