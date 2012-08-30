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


#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#	pkg_check_modules(GAZEBO_SERVER REQUIRED gazebo)
#	pkg_check_modules(GAZEBO_ODE REQUIRED gazebo_ode)
#	pkg_check_modules(GAZEBO_TRANSPORT REQUIRED gazebo_transport)
#pkg_check_modules(GAZEBO gazebo)
#endif()
#include_directories(${GAZEBO_INCLUDE_DIRS})
#link_directories(${GAZEBO_LIBRARY_DIRS})

rosbuild_add_library(hello_world SHARED src/hello_world.cpp)
rosbuild_add_library(world_plugin_template SHARED src/world_plugin_template.cpp)
rosbuild_add_library(inverted_pendulum SHARED src/inverted_pendulum.cpp)
rosbuild_add_library(inverted_pendulum_with_foot SHARED src/inverted_pendulum_with_foot.cpp)
rosbuild_add_library(ground_contact SHARED src/ground_contact.cpp)
rosbuild_add_library(atrias20_leg SHARED src/atrias20_leg.cpp)
rosbuild_add_library(freeze_pose SHARED src/freeze_pose.cpp)
rosbuild_add_library(pause_world SHARED src/pause_world.cpp)
rosbuild_add_library(spring SHARED src/spring.cpp)
rosbuild_add_library(atrias20_biped SHARED src/atrias20_biped.cpp)
rosbuild_add_library(planar_constraint SHARED src/planar_constraint.cpp)
