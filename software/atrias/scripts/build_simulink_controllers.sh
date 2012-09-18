#!/bin/bash

# Where is MATLAB located?
localMatlabPath="/usr/local/MATLAB/R2012a"
if [ ! -f "${localMatlabPath}/bin/matlab" ]; then
    echo
    echo "-----------ERROR!-----------"
    echo "Referencing MATLAB from here: $localMatlabPath"
    echo "But matlab could not be located"
    echo "Correct the localMatlabPath variable in this script to continue"
    echo
    exit
fi

# Change to the simulink_controllers directory
cd "${0%/*}/../../atrias_controllers/simulink_controllers"
simulinkControllersDir=$(pwd)

# Move any .zip downloaded files with the correct controller suffix to the simulink_controller directory
downloadFiles=( $(ls ~/Downloads) )
for download in ${downloadFiles[@]}
do
    controllerZip=$(echo $download | grep "_ert_shrlib_rtw-[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9].zip$")
    # If it is a controller
    if [ "$controllerZip" != "" ]
    then
        # Remove the current controller if it exists
        oldControllerDir=$(echo $controllerZip | sed "s/-[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9].zip$//")
        oldControllerName=$(echo $oldControllerDir | sed "s/_ert_shrlib_rtw//")
        rm -f -r "$oldControllerDir"
        rm -f "lib${oldControllerName}.so"
        # Move and extract the new controller
        mv ~/Downloads/$download .
        unzip -q "$download"
        rm -f "$download"
        rm -f ~/Downloads/$download
        # Feedback
        echo
        echo "Note:"
        echo "Extracted $download to simulink_controllers"
        echo
    fi
done

# Pick a controller
controllerList=$(find . -mindepth 1 -maxdepth 1 -type d | sed 's|^./||')
while [ 1 ]; do
    echo "Simulink controllers:"
    echo "$controllerList" | sed "s/_ert_shrlib_rtw//" | nl -w1 -s") "
    echo
    echo "What controller do you want to make?"
    read controllerNumber

    maxNumber=$(echo "$controllerList" | wc -l)
    if [[ "$controllerNumber" > "$maxNumber" || "$controllerNumber" < 1 ]]; then
        echo
        echo "ERROR: Invalid controller number"
        echo
    else
        controllerDir=$(echo "$controllerList" | sed -n ${controllerNumber}p)
        break
    fi

done

# Convert the makefile to the local machine
cd "$controllerDir"
controllerMakefile=$(ls | grep mk$ )
controllerName="${controllerMakefile%.*}"
sed -i "s|^MAKECMD         =.*|MAKECMD         = ${localMatlabPath}/bin/glnxa64/gmake|" "${controllerMakefile}"
sed -i "s|^MATLAB_ROOT             =.*|MATLAB_ROOT             = ${localMatlabPath}|" "${controllerMakefile}"
sed -i "s|^ALT_MATLAB_ROOT         =.*|ALT_MATLAB_ROOT         = ${localMatlabPath}|" "${controllerMakefile}"
sed -i "s|^START_DIR               =.*|START_DIR               = ${simulinkControllersDir}|" "${controllerMakefile}"

# Do we want to make the file?
echo "Controller makefile converted.  Press enter to make clean and make the controller"
read

# Clean up previous compiles
rm -f *.o
rm -f ../$controllerName
rm -f ../${controllerName}.so
rm -f ../lib${controllerName}.so

# make the controller
make -f ${controllerMakefile}

# Move the shared library to where ROS expects it
cd ..
mv "${controllerName}.so" "lib${controllerName}.so"

# Remove the example main program
rm -f $controllerName
