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
templates="${0%/*}/templates"
# cd to the atrias_controllers path
cd ${0%/*/*/*}/atrias_controllers

# Determine what controller the user wants to create
while [ 1 ]; do
    echo "What type of controller do you want to create?"
    echo "1) atc"
    echo "2) asc (plugin)"
    echo "3) asc (component)"

    read arg

    case "$arg" in
    1) 
        atc=1
        ascPlugin=0
        ascComponent=0
        break
        ;;
    2)
        atc=0
        ascPlugin=1
        ascComponent=0
        break
        ;;
    3)
        atc=0
        ascPlugin=0
        ascComponent=1
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
        read name[count]

        clear
        echo "Current subcontroller list:"
        echo "${name[@]}" | sed "s/ /\n/g"
        echo
        ;;
    2)
        unset name
        count=-1
        clear
        ;;
    3)
        # Make sure the subcontroller names are valid
        validNames=0
        for var in ${name[@]}; do
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
            echo "${name[@]}" | sed "s/ /\n/g"
            echo
        fi
        ;;
    *)
        echo "Invalid input."
        ;;
    esac
done

# Generate the files
