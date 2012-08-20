#!/bin/bash

# Move rosbag and all of its subthreads to the non-realtime core.
while [ -z $rosbag_present ]; do
    rosbag_present=`rosnode list | grep 'atrias_rosbag'`
done

rosbagpid=`echo \`pidof record\` | sed 's/ /,/g'`
roslaunchpid=`echo \`pidof python\` | sed 's/ /,/g'`
sudo cset proc -m --threads ${rosbagpid} user
sudo cset proc -m --threads ${roslaunchpid} user

exit 0
