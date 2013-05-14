#!/bin/bash

# This script cleans all controllers. This is necessary after a change in the controller library
# because otherwise they have linking issues that leads to segfaults (particularly on unload)

# Confirm we're in the right directory
if [ `basename $PWD` != atrias_controllers ]
then
	echo "This script must be run from the atrias_controllers directory. Exiting."
	exit 4
fi

for package in asc_* atc_*
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

	# Back to atrias_controllers
	cd ..
done

# Notify the user they have to build the controllers manually
echo "Controllers cleaned. Please manually build all desired controllers."
