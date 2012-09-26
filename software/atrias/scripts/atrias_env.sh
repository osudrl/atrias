#!/bin/bash

# ROS
. /opt/ros/fuerte/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/home/drl/rosstacks:/home/drl/atrias/software/
export ROS_MASTER_URI=http://drl-guilaptop:11311

# Orocos
. `rosstack find orocos_toolchain`/env.sh
export OROCOS_TARGET=xenomai

# Xenomai
export XENOMAI_ROOT_DIR=/usr/xenomai
export PATH="$PATH:/usr/xenomai/bin/"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:/usr/xenomai/lib/"

exec "$@"
