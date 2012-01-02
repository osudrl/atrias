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

include_directories(/opt/etherlab/include)

#link_directories(/opt/etherlab/lib)

# Get ECAT compiler flags.
#set(ECAT_CFLAGS "-I /opt/etherlab/include -L /opt/etherlab/lib -l ethercat")
#message("ECAT FLAGS: ${ECAT_CFLAGS}")

# Add ECAT compiler flags.
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ECAT_CFLAGS}")
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")
#message("CFLAGS: ${CMAKE_CXX_FLAGS}")

rosbuild_add_executable(uspace_controller src/uspace_controller_wrapper.cpp)

# Link libraries

find_library(ECAT_LIBRARY ethercat PATH /opt/etherlab/lib)
target_link_libraries(uspace_controller ${ECAT_LIBRARY})

#rosbuild_link_boost(uspace_controller thread)

