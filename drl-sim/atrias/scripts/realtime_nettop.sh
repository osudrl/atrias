#!/bin/bash

if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root." 1>&2
   echo "Please run via sudo instead." 1>&2
   exit 1
fi

(echo 0 ; while [ true ] ; do sleep 1 ; done) > /dev/cpu_dma_latency &
CPU_DMA_LATENCY_PID="$!"

cset set -c 1,3 user
cset set -c 0,2 system
cset proc -k on --force -t system -f root

cset shield -e roslaunch -- atrias nettop.launch

cset proc -k on --force -t root -f system
cset proc -k on --force -t root -f user
cset set -d user
cset set -d system

kill -n 9 $CPU_DMA_LATENCY_PID
