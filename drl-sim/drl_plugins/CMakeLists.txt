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

rosbuild_add_library(spring src/spring.cpp)
rosbuild_link_boost(spring thread)

rosbuild_add_library(joint_impedance src/joint_impedance.cpp)
rosbuild_link_boost(joint_impedance thread)

rosbuild_add_library(position_body src/position_body.cpp)
rosbuild_link_boost(position_body thread)

rosbuild_add_library(invertedPendulum src/invertedPendulum.cpp)
rosbuild_link_boost(invertedPendulum thread)

rosbuild_add_library(invertedPendulumWithToe src/invertedPendulumWithToe.cpp)
rosbuild_link_boost(invertedPendulumWithToe thread)
