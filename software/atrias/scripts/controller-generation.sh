#!/bin/bash

# This code makes a few assumptions:
# 1) top-level controllers are prefixed with atc_
# 2) sub-level controllers are prefixed with asc_
# 3) sub-level controllers are located in atrias_controllers
# 4) there is only one orocos component per subcontroller package

function help
{
    echo
    echo "Creates and links top-level and sub-level controllers for use with atrias."
    echo
}

# check to see if there is a help flag
if [[ "$*" = "-h" || "$*" = "--help" ]]
then
    help
    exit 1
fi

# The current directory
cwd="$(pwd)"
# The template path
templates="${0%/*}/../templates"
cd $templates
templates="$(pwd)"
cd $cwd
# The atrias_controllers path
atriasControllers="${0%/*}/../../atrias_controllers"
cd $atriasControllers
atriasControllers="$(pwd)"

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
        echo "Do you want this controller to have an output port to the gui? (y/n)"
        read guiOutDecision
        while [[ "$guiOutDecision" != "y" && "$guiOutDecision" != "n" ]]
        do
            echo "Unrecognized answer.  y/n?"
            read guiOutDecision
        done
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
namesMasterList=( $(ls | grep asc_) )
while [ 1 ]; do
    echo "Do you want to link to existing subcontrollers?"
    echo "1) Add a subcontroller"
    echo "2) Clear the subcontroller list"
    echo "3) Done"

    read arg
    case "$arg" in
    1) 
        echo "Existing subcontrollers in atrias_controllers:"
        echo "${namesMasterList[@]}" | sed "s| |\n|g" | cat -n
        echo
        (( count += 1 ))
        echo "Enter the number corresponding to the subcontroller"
        read subcontrollerNumber
        names[count]=$(echo "${namesMasterList[@]}" | sed 's| |\n|g' | cat -n | grep ${subcontrollerNumber} | sed "s|^.*\\t||")

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
        for var in ${names[@]}
        do
            ls | grep "asc" | grep "^${var}$" &> /dev/null
            validNames=$?

            if [ "$validNames" = "1" ]
            then
                break
            fi
        done
        
        # If there there were no invalid names, then break.  Else, warn and loop
        if [ $validNames = 0 ]
        then
            break
        else
            clear
            echo "ERROR: A subcontroller name is invalid. (This only happens if the code is broken)"
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



###### Generate the files ######

# For a top-level controller
if [[ -n $atcName ]]
then
    # Figure out names
    lowerScoredName="$atcName"
    upperCamelName=$(echo $lowerScoredName | sed "s|\(^...\)|\U\1|; s|_\(.\)|\U\1|g")
    lowerCamelName=$(echo $upperCamelName | sed "s|\(^...\)|\L\1|")
    # Copy the template and change to the new directory
    cd ${cwd}
    cp -r "${templates}/atc_component" "${cwd}/${lowerScoredName}"
    cd ${lowerScoredName}
    # Move the include folder to its appropriate location
    mv include/atc_component include/${lowerScoredName}
    # Rename things
    files=( start.ops manifest.xml mainpage.dox CMakeLists.txt src/controller_component.cpp src/controller_gui.cpp include/${lowerScoredName}/controller_component.h include/${lowerScoredName}/controller_gui.h )
    for file in ${files[@]}
    do
        sed -i "s|atc_template|${lowerScoredName}|g; s|ATCTemplate|${upperCamelName}|g" $file
    done
    # Controller description
    echo "description=${description}" > controller.txt
    # Clean up the existing '.svn' folders
    find . -name '.svn' -exec rm -rf {} +

    # Add subcontroller code
    for name in ${names[@]}
    do
        cd ${atriasControllers}/${name}
        component=$(grep "^orocos_component(" CMakeLists.txt)
        service=$(grep "^orocos_service(" CMakeLists.txt)
        if [ "$component" != "" ]
        then
            # Get the component name
            componentName=$(grep "ORO_CREATE_COMPONENT" src/controller_component.cpp | sed 's|ORO_CREATE_COMPONENT(||; s|).*||')
            # Get the unique name suffix
            uniqueNameSuffix=$(echo ${names} | sed "s|^....||; s|_\(.\)|\U\1|g")
            # Get a unique name
            cd ${cwd}/${lowerScoredName}
            uniqueNumber=0
            unique=${uniqueNameSuffix}${uniqueNumber}
            uniqueName=${uniqueNameSuffix}${uniqueNumber}Name
            uniqueController=${uniqueNameSuffix}${uniqueNumber}Controller
            while [ "$(grep "$uniqueName" start.ops)" != "" ]
            do
                (( uniqueNumber += 1 ))
                unique=${uniqueNameSuffix}${uniqueNumber}
                uniqueName=${uniqueNameSuffix}${uniqueNumber}Name
                uniqueController=${uniqueNameSuffix}${uniqueNumber}Controller
            done

            # Add the component to the start script
            mv start.ops start.ops.old
            if [ "$uniqueNumber" -eq "0" ]
            then
                cat start.ops.old | sed -n '1!N; s|# Set up subcontrollers|# Set up subcontrollers\nimport("'$name'")\nvar string '$uniqueName' = atrias_cm.getUniqueName("controller", "'$uniqueNameSuffix'")\nloadComponent('$uniqueName', "'$componentName'")|; p' > start.ops
            else
                cat start.ops.old | sed -n '1!N; s|import("'$name'")|import("'$name'")\nvar string '$uniqueName' = atrias_cm.getUniqueName("controller", "'$uniqueNameSuffix'")\nloadComponent('$uniqueName', "'$componentName'")|; p' > start.ops
            fi
            rm start.ops.old
            mv start.ops start.ops.old
            cat start.ops.old | sed -n '1!N; s|# Connect this controller with its subcontrollers|# Connect this controller with its subcontrollers\naddPeer("controller", '$uniqueName')|; p' > start.ops
            rm start.ops.old
            mv start.ops start.ops.old
            cat start.ops.old | sed -n '1!N; s|# Pass the names of the subcontrollers to the controller|# Pass the names of the subcontrollers to the controller\ncontroller.'$uniqueName' = '$uniqueName'|; p' > start.ops
            rm start.ops.old

            # Add the component to the .cpp file
            cd src
            mv controller_component.cpp controller_component.cpp.old
            cat controller_component.cpp.old | sed -n '1!N; s|// Add properties|// Add properties\n    this->addProperty("'$uniqueName'", '$uniqueName')\n        .doc("Subcontroller name.");|; p' > controller_component.cpp
            rm controller_component.cpp.old
########################################################################################################
            cd ${atriasControllers}/${name}
            services=( $(grep 'this->provides(' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
            for service in ${services[@]}
            do
                operations=( $(grep 'addOperation("' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
                for operation in ${operations[@]}
                do
                    mv controller_component.cpp controller_component.cpp.old
                    cat controller_component.cpp.old | sed -n '1!N; s|// Connect to the subcontrollers|// Connect to the subcontrollers\n    '$unique' = this->getPeer('$uniqueName');\n    if ('$unique')\n        '$uniqueController' = '$unique'->provides("'$service'")->getOperation("'$operation'");|; p' > controller_component.cpp
                    rm controller_component.cpp.old
                done
            done
            # Add references to attributes
            # ...
########################################################################################################

            # Add the component to the .h file

        elif [ "$service" != "" ]
        then
            # Find the service files
            srcFiles=( $(ls src) )
            serviceNames=""
            # For each file, find its service and operations
            for file in ${srcFiles[@]}
            do
                cd ${atriasControllers}/${name}
                serviceName=$(cat src/$file | grep 'Service(' | sed "s|.*(\"||; s|\".*||")
                # Add the service to the start script
                cd ${cwd}/${lowerScoredName}
                mv start.ops start.ops.old
                cat start.ops.old | sed -n '1!N; s|# Set up subcontrollers|# Set up subcontrollers\nrequire("'$serviceName'")\nloadService("controller", "'$serviceName'")|; p' > start.ops
                rm start.ops.old

                # Find the operations for this file
                cd ${atriasControllers}/${name}
                operationNames=( $(cat src/$file | grep 'addOperation(' | sed "s|.*addOperation(\"||; s|\".*||") )
                for operation in ${operationNames[@]}
                do
                    # Find the operation's returnVar and arguments
                    cd ${atriasControllers}/${name}/src
                    callbackFunction=$(cat $file | grep "addOperation(\"$operation" | sed 's|.*::||; s|,.*||')
                    args=$(cat $file | grep "::${callbackFunction}(" | sed "s|.*(||; s|).*||; s|\(.* \).*|\1|; s| .*,|,|g; s| $||")
                    returnVar=$(cat $file | grep "::${callbackFunction}(" | sed "s| .*||")
                    
                    # Add the operation to the .ccp file
                    cd ${cwd}/${lowerScoredName}/src
                    mv controller_component.cpp controller_component.cpp.old
                    cat controller_component.cpp.old | sed -n '1!N; s|// Service plugins|// Service plugins\n    '$operation' = this->provides("'$serviceName'")->getOperation("'$operation'");|; p' > controller_component.cpp
                    rm controller_component.cpp.old
                    
                    # Add the operation to the .h file
                    cd ../include/${lowerScoredName}
                    mv controller_component.h controller_component.h.old
                    cat controller_component.h.old | sed -n '1!N; s|// Service plugins|// Service plugins\n    OperationCaller<'$returnVar'('"$args"')> '$operation';|; p' > controller_component.h
                    rm controller_component.h.old
                    cd ../..  # Go back to the package directory
                done
            done
        else
            echo "ERROR: subcontroller ${name} not recognized as a service plugin or component"
        fi

    done

    # Clean up the template
    cd ${cwd}/${lowerScoredName}
    sed -i 's|# ============ Needed for template creation ============||' start.ops

    # Do we want a gui output port?
    if [ "$guiOutDecision" = "n" ]
    then
        mv start.ops start.ops.old
        grep -v "gui_policy.*status" start.ops.old | grep -v "stream.*gui_data_out" > start.ops
        rm start.ops.old

        cd src
        mv controller_component.cpp controller_component.cpp.old
        grep -v "guiDataOut" controller_component.cpp.old | grep -v "Send data to the GUI" | grep -v "readyToSend" > controller_component.cpp
        rm controller_component.cpp.old
        cd ..

        cd include/${lowerScoredName}
        mv controller_component.h controller_component.h.old
        grep -v "guiOut" controller_component.h.old | grep -v "guiDataOut" > controller_component.h
        rm controller_component.h.old
        cd ../..

        rm msg/controller_log_data.msg
    fi


# For a sub-controller plugin
elif [[ -n $ascPluginName ]]
then
    name="$ascPluginName"
    #cp -r ${templates}/asc_service_plugin ${cwd}/${name}
    echo "Dummy command"


# For a sub-controller component
elif [[ -n $ascComponentName ]]
then
    name="$ascComponentName"
    echo "Dummy command"
    #cp -r ${templates}/asc_component ${cwd}/${name}
else
    echo "No controller name specified"
    exit 1
fi


echo "Controller generated"
exit 0
