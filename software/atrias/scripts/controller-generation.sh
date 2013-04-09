#!/bin/bash

# Abort upon encountering an error
set -e

# Confirm we're in the right directory
if [ `basename $PWD` != atrias_controllers ]
then
	echo "This script must be run from the atrias_controllers directory. Exiting."
	exit 4
fi

# Read in the desired package name from the user
echo -n "Please input the desired package name, such as asc_pd or atc_motor_torque: "
read PACKAGE_NAME

# Extract the "atc" or "asc" from the package name
PACKAGE_TYPE=${PACKAGE_NAME:0:3}

if [ "$PACKAGE_TYPE" == "atc" ]
then
	echo "Generating top-level controller."
elif [ "$PACKAGE_TYPE" == "asc" ]
then
	echo "Generating subcontroller."
else
	echo "The package name must begin with asc_ or atc_. Exiting"
	exit 1
fi

# Check if the controller already exists
if [ -e $PACKAGE_NAME ]
then
	echo "$PACKAGE_NAME already exists! Exiting."
	exit 2
fi

# Now that we've verified we won't be overwriting existing files, copy over the template
echo "Copying over template."
cp -r ../atrias/templates/$PACKAGE_TYPE $PACKAGE_NAME

# Change into the controller directory
cd $PACKAGE_NAME

# Move over the include directory, rename files
mv include/ToSubstitutePackageName include/$PACKAGE_NAME

# Ask the user for the name of the main class
echo -n "Please input the desired class name, such as ASCPD or ATCMotorTorque: "
read CLASS_NAME

# Rename the class files
mv include/$PACKAGE_NAME/ToSubstituteClassName.hpp include/$PACKAGE_NAME/$CLASS_NAME.hpp
mv src/ToSubstituteClassName.cpp src/$CLASS_NAME.cpp

# Execute search and replaces
# The find command lists all the normal files within this directory or its subdirectories.
# The -i flag to sed instructs it to edit the files in-place
# The 's/ToSubstitutePackageName/$PACKAGE_NAME/g' tells it to substitute in the package name
# and to do so multiple times if "ToSubstitutePackageName" appears multiple times on one line
sed "s/ToSubstitutePackageName/$PACKAGE_NAME/g" -i `find . -type f`
sed "s/ToSubstituteClassName/$CLASS_NAME/g" -i `find . -type f`

# Substitute in the description
echo "Please enter a description of this controller:"
read DESCRIPTION
sed "s/ToSubstituteDescription/$DESCRIPTION/g" -i `find . -type f`

echo "Controller creation complete."

exit 0

# vim: noexpandtab
