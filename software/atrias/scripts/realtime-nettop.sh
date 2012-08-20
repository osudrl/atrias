#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root." 1>&2
   echo "Please run via sudo instead." 1>&2
   exit 1
fi

(echo 0 ; while [ true ] ; do sleep 1 ; done) > /dev/cpu_dma_latency &
CPU_DMA_LATENCY_PID="$!"

# Give our (userspace) process access to all cores and limit the system
# processes to core 0. We will set the CPU affinity of the controller manager
# to 0 in Orocos scripting so the RT operations manager can run by itself on
# core 1.
cset set -c 0,2 user
cset set -c 0,2 system
cset proc -k on --force -m -t system -f root

#cset shield -e roslaunch -- atrias orocos-nogui.launch 
cset proc -e user roslaunch -- atrias orocos-nogui.launch
#cset proc -e user rosrun -- ocl deployer-gnulinux -s `rospack find atrias_controller_manager`/controller_manager.ops -l info

cset proc -k on --force -m -t root -f system
cset set -d user

kill -n 9 $CPU_DMA_LATENCY_PID
