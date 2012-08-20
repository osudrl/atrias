#!/bin/bash
# Andrew Peekema
# Creates a new template simulation including:
# a launch file
# a world file
# an h file
# a cpp file
# adds the cpp file to CMakeLists.txt

name=${1}

if [ "${name}" == "" ]; then
  echo "Usage:"
  echo ". ./create_new_simulation.sh simulation_name"
  exit 1
fi


# Change to the package root
roscd atrias_sim

# If roscd doesn't work, you're doing it wrong
if [ ${?} != 0 ]; then
  echo "Make sure the command starts with \". ./\""
  exit 1
fi

# Launch file
cd launch
cp world_plugin_template.launch ${name}.launch
sed -i "s/world_plugin_template/${name}/g" ${name}.launch

# World file
cd ../worlds
cp world_plugin_template.world ${name}.world
sed -i "s/world_plugin_template/${name}/g" ${name}.world

# h file
cd ../include/atrias_sim
cp world_plugin_template.h ${name}.h
sed -i "s/WORLD_PLUGIN_TEMPLATE/${name^^}/g" ${name}.h

# cpp file
cd ../../src
cp world_plugin_template.cpp ${name}.cpp
sed -i "s/world_plugin_template/${name}/g" ${name}.cpp

# CMakeLists.txt
cd ..
echo "rosbuild_add_library(${name} SHARED src/${name}.cpp)" >> CMakeLists.txt


