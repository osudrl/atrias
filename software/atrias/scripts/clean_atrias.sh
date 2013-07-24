#!/bin/bash

# This script cleans the atrias repo.

# Source bashrc so we can run ROS commands. 
# Note: The PS1 setting is a hack to make bashrc actually run
PS1='$ '
. ~/.bashrc

roscd atrias/..

# Confirm we're in the right directory
if [ `basename $PWD` != software ]
then
	echo "This script must be run from the atrias_software directory. Exiting."
	exit 4
fi

for package in atrias_*
do
	# Change into this directory
	cd $package
	echo "Cleaning $package"

	# Standard cleanup (if the directory exists
	if [ -d build ]
	then
		make --no-print-directory clean
	fi

	# Clean up some other files that also mess with the build
	rm -f $package*.pc
	rm -rf bin lib msg_gen

	# Back to software
	cd ..
done

# Notify the user they have to build the repo manually
echo "Atrias repo cleaned. Please manually rebuild."
