#!/bin/bash

function help
{
    echo
    echo "Creates and links top-level and sub-level controllers for use with atrias."
    echo
}

# check to see if there is a help flag
if [ "$*" = "-h" ] || [ "$*" = "--help" ]; then
    help
    exit 1
fi

# The current directory
cwd="$(pwd)"
# The template directory
templates="${0%/*}/../templates"
# cd to the atrias_controllers path
cd ${0%/*}/../../atrias_controllers

# Determine what controller the user wants to create
while [ 1 ]; do
    echo "What type of controller do you want to create?"
    echo "1) atc"
    echo "2) asc (plugin)"
    echo "3) asc (component)"

    read arg

    case "$arg" in
    1) 
        echo "What do you want to name it? (e.g. atc_example_name)"
        read atcName
        echo "Give a description of this controller"
        read description
        break
        ;;
    2)
        echo "What do you want to name it? (e.g. asc_example_name)"
        read ascPluginName
        echo "Give a description of this controller"
        read description
        break
        ;;
    3)
        echo "What do you want to name it? (e.g. asc_example_name)"
        read ascComponentName
        echo "Give a description of this controller"
        read description
        break
        ;;
    *)
        echo "Not a valid selection.  Please select 1, 2, or 3."
        ;;
    esac
done

# Determine what subcontrollers the user wants to link to
clear
count=-1
while [ 1 ]; do
    echo "Existing subcontrollers in atrias_controllers:"
    ls | grep asc_
    echo
    echo "Do you want to link to existing subcontrollers?"
    echo "1) Add a subcontroller"
    echo "2) Clear the subcontroller list"
    echo "3) Done"

    read arg
    case "$arg" in
    1) 
        (( count += 1 ))
        echo "Enter subcontroller name"
        read names[count]

        clear
        echo "Current subcontroller list:"
        echo "${names[@]}" | sed "s/ /\n/g"
        echo
        ;;
    2)
        unset names
        count=-1
        clear
        ;;
    3)
        # Make sure the subcontroller names are valid
        validNames=0
        for var in ${names[@]}; do
            ls | grep "asc" | grep "^${var}$" &> /dev/null
            validNames=$?
            if [ "$validNames" = "1" ]; then
                break
            fi
        done
        
        # If there there were no invalid names, then break.  Else, warn and loop
        if [ $validNames = 0 ]; then
            break
        else
            clear
            echo "ERROR: A subcontroller name is invalid."
            echo
            echo "Current subcontroller list:"
            echo "${names[@]}" | sed "s/ /\n/g"
            echo
        fi
        ;;
    *)
        echo "Invalid input."
        ;;
    esac
done

# Double-check this is what we want to do
clear
echo "This script will create the package ${atcName}${ascPluginName}${ascComponentName} in this directory with these subcontrollers: ${names[@]}"
echo "If this is what you want to do, press Enter..."
read

# Generate the files
if [[ -n $atcName ]]; then
    # Figure out names
    lowerScoredName="$atcName"
    upperCamelName=$(echo $atcName | sed "s|\(^...\)|\U\1|; s|_\(.\)|\U\1|g")
    lowerCamelName=$(echo $upperCamelName | sed "s|\(^...\)|\L\1|")
    # Copy the template and change to the new directory
    cp -r ${templates}/atc_component ${cwd}/${name}
    cd ${cwd}/${name}
    # Move the include folder to its appropriate location
    mv include/atc_component include/${lowerScoredName}
    # Rename things
    files=( start.ops manifest.xml mainpage.dox CMakeLists.txt src/controller_component.cpp src/controller_gui.cpp include/${lowerScoredName}/controller_component.h include/${lowerScoredName}/controller_gui.h )
    for file in ${files[@]}; do
        sed -i "s|atc_template|${lowerScoredName}|g; s|ATCTemplate|${upperCamelName}|g" file
    done
    # Specific substitutions
    echo "description=${description}" > controller.txt
    # Clean up the '.svn' folders in the directories
    find . -name '.svn' -exec rm -rf {} +

elif [[ -n $ascPluginName ]]; then
    name="$ascPluginName"
    #cp -r ${templates}/asc_service_plugin ${cwd}/${name}
    echo "Dummy command"
elif [[ -n $ascComponentName ]]; then
    name="$ascComponentName"
    echo "Dummy command"
    #cp -r ${templates}/asc_component ${cwd}/${name}
else
    echo "No controller name specified"
    exit 1
fi


echo "Controller generated"
exit 0
