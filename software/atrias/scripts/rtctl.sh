#!/bin/bash

usage() {
  echo "Usage: rtctl command"
  echo "Commands:"
  echo "start   - Unload e1000e, load rtnet"
  echo "stop    - Unload rtnet, load e1000e"
  echo "restart - Unload rtnet, reset interface, reload rtnet"
  exit 1
}

removert() {
  # Remove rtnet modules
  /usr/local/rtnet/sbin/rtifconfig rteth0 down
  modprobe -r rt_e1000e
  modprobe -r rtpacket
  modprobe -r rtnet
}

loadeth() {
  # Load then unload e1000e to reset interface
  # Load e1000e
  modprobe e1000e
  service network-interface restart INTERFACE=eth0
  ifconfig eth0 up
}

rmeth() {
  rmmod e1000e
  ifconfig eth0 down
}

loadrt() {
  # Load rtnet modules
  modprobe rtnet
  modprobe rtpacket
  modprobe rt_e1000e
  /usr/local/rtnet/sbin/rtifconfig rteth0 up
}



if [ "$(id -u)" != "0" ]; then
	echo "This script requires root"
	exit 1
fi

if [ -z "$1" ];
then
  usage

elif [ $1 = 'restart' ]; then
  removert
  loadeth
  rmeth
  loadrt
  exit 0

elif [ $1 = 'stop' ]; then
  removert
  loadeth
  exit 0

elif [ $1 = 'start' ]; then
  rmeth
  loadrt
  exit 0

else
  usage
fi
