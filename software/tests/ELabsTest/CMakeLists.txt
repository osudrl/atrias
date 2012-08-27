cmake_minimum_required(VERSION 2.6.3)
project(atrias_elabs_test)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package( rtt )
set( RTT_HINTS HINTS ${rtt_PACKAGE_PATH}/../install )

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

include_directories(../../robot_definitions/)
include_directories(/opt/etherlab/include)
find_library(ECAT_LIBRARY ethercat PATH /opt/etherlab/lib)
orocos_component(ELabsTest src/ELabsTest.cpp)
target_link_libraries(ELabsTest ${ECAT_LIBRARY})

orocos_generate_package()
