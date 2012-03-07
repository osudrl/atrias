#!/bin/bash
# Pauses physics in an active Gazebo simulation
paused=0
while [ ${paused} == 0 ]
do
	gazebo=`rosnode list | grep gazebo`
	
	if [ -n "${gazebo}" ]; then
		rosservice call /gazebo/pause_physics
		paused=1
	fi
done
