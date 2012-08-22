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

function sedMultiLine
{
    file="$1"
    find="$2"
    replace="$3"
    mv "$file" "${file}.old"
    sed -n "1!N; s|$find|$replace|g; p" "${file}.old" > "$file"
    rm "${file}.old"
}

function grepRemoveLines
{
    # Usage:  grepRemoveLines "file" "line1" "line2" ...
    file="$1"
    shift  # This shifts the input variable over one

    while (( "$#" ))  # While there are input variables
    do
        mv "$file" "${file}.old"
        grep -v "$1" "${file}.old" > "$file"
        rm "${file}.old"
        shift
    done
}

# check to see if there is a help flag
if [[ "$*" = "-h" || "$*" = "--help" ]]
then
    help
    exit 1
fi

# The current path
currentPath="$(pwd)"
# The templates path
templatesRelativePath="${0%/*}/../templates"
cd "$templatesRelativePath"
templatesPath="$(pwd)"
# The atrias_controllers path
atriasControllersRelativePath="${0%/*}/../../atrias_controllers"
cd "$currentPath"
cd "$atriasControllersRelativePath"
atriasControllersPath="$(pwd)"


################# Get User Input #################

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
        read newAtcName
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
        read newAscPluginName
        echo "Give a description of this controller"
        read description
        break
        ;;
    3)
        echo "What do you want to name it? (e.g. asc_example_name)"
        read newAscComponentName
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
cd "$atriasControllersPath"
ascList=( $(ls | grep asc_) )
while [ 1 ]; do
    echo "Do you want to link to existing subcontrollers?"
    echo "1) Add a subcontroller"
    echo "2) Clear the subcontroller list"
    echo "3) Done"

    read arg
    case "$arg" in
    1) 
        echo "Existing subcontrollers in atrias_controllers:"
        echo "${ascList[@]}" | sed "s| |\n|g" | cat -n
        echo
        echo "Enter the number corresponding to the subcontroller"
        read ascNumber
        ascsToLink=( ${ascsToLink[@]} $(echo "${ascList[@]}" | sed 's| |\n|g' | sed -n ${ascNumber}p) )

        clear
        echo "Current subcontroller list:"
        echo "${ascsToLink[@]}" | sed "s| |\n|g"
        echo
        ;;
    2)
        unset ascsToLink
        clear
        ;;
    3)
        break
        ;;
    *)
        echo "Invalid input."
        ;;
    esac
done

# Double-check this is what we want to do
clear
echo "This script will create the package ${newAtcName}${newAscPluginName}${newAscComponentName} in this directory with these subcontrollers:"
echo "${ascsToLink[@]-None}" | sed "s| |\n|g"
echo
echo "If this is what you want to do, press Enter..."
read



################# Generate the files #################

# For a top-level controller
if [[ -n "$newAtcName" ]]
then
    # Figure out names
    newAcName="$newAtcName"
    newAcCamelName=$(echo $newAcName | sed "s|\(^...\)|\U\1|; s|_\(.\)|\U\1|g")
    newAcPath="${currentPath}/${newAcName}"
    # Copy the template and change to the new directory
    cp -r "${templatesPath}/atc_component" "$newAcPath"
    # Sanity check (This directory must exist)
    if [ ! -d $newAcPath ]
    then
        echo "ERROR: Could not copy from the templates directory"
        exit 1
    fi
    cd "$newAcPath"
    # Move the include folder to its appropriate location
    mv "include/atc_component" "include/$newAcName"
    # Rename things
    files=( start.ops manifest.xml mainpage.dox CMakeLists.txt src/controller_component.cpp src/controller_gui.cpp include/${newAcName}/controller_component.h include/${newAcName}/controller_gui.h )
    for file in ${files[@]}
    do
        sed -i "s|atc_template|${newAcName}|g; s|ATCTemplate|${newAcCamelName}|g" $file
    done
    # Controller description
    echo "description=$description" > controller.txt
    # Clean up the existing '.svn' folders
    find . -name '.svn' -exec rm -rf {} +

    # Add subcontroller code
    for ascToLinkName in ${ascsToLink[@]}
    do
        ascToLinkPath=${atriasControllersPath}/${ascToLinkName}
        cd "$ascToLinkPath"
        ascIsAComponent=$(grep "^orocos_component(" CMakeLists.txt)
        ascIsAService=$(grep "^orocos_service(" CMakeLists.txt)

        # If it's a component
        if [ "$ascIsAComponent" != "" ]
        then
            # Get the component name
            componentName=$(grep "ORO_CREATE_COMPONENT" src/controller_component.cpp | sed 's|ORO_CREATE_COMPONENT(||; s|).*||')
            # Get the unique name suffix
            uniqueNamePrefix=$(echo ${ascToLinkName} | sed "s|^....||; s|_\(.\)|\U\1|g")
            # Get a unique name
            uniqueNumber=0
            unique=${uniqueNamePrefix}${uniqueNumber}
            cd "${newAcPath}"
            while [ "$(grep "${unique}Name" start.ops)" != "" ]
            do
                (( uniqueNumber += 1 ))
                unique=${uniqueNamePrefix}${uniqueNumber}
            done

            # Add the component to the start script
            file='start.ops'
            if [ "$uniqueNumber" -eq "0" ]
            then
                find='# Set up subcontrollers'
                replace='# Set up subcontrollers\nimport("'$ascToLinkName'")\nvar string '${unique}Name' = atrias_cm.getUniqueName("controller", "'${uniqueNamePrefix}'")\nloadComponent('${unique}Name', "'$componentName'")\n'
            else
                find='import("'$ascToLinkName'")'
                replace='import("'$ascToLinkName'")\nvar string '${unique}Name' = atrias_cm.getUniqueName("controller", "'${uniqueNamePrefix}'")\nloadComponent('${unique}Name', "'$componentName'")'
            fi
            sedMultiLine "$file" "$find" "$replace"

            find='# Connect this controller with its subcontrollers'
            replace='# Connect this controller with its subcontrollers\naddPeer("controller", '${unique}Name')'
            sedMultiLine "$file" "$find" "$replace"

            find='# Pass the names of the subcontrollers to the controller'
            replace='# Pass the names of the subcontrollers to the controller\ncontroller.'${unique}Name' = '${unique}Name''
            sedMultiLine "$file" "$find" "$replace"

            # Add the component to the .cpp file
            file='src/controller_component.cpp'
            find='// Add properties'
            replace='// Add properties\n    this->addProperty("'${unique}Name'", '${unique}Name')\n        .doc("Subcontroller name.");'
            sedMultiLine "$file" "$find" "$replace"
            cd "$ascToLinkPath"
            services=( $(grep 'this->provides(' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
            for service in ${services[@]}
            do
                operations=( $(grep 'addOperation("' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
                cd "$newAcPath"
                for operation in ${operations[@]}
                do
                    find='// Connect to the subcontrollers'
                    operationReference=$(echo "${unique}_${operation}" | sed "s|_\(.\)|\U\1|g")
                    replace='// Connect to the subcontrollers\n    '$unique' = this->getPeer('${unique}Name');\n    if ('$unique')\n        '${operationReference}' = '$unique'->provides("'$service'")->getOperation("'$operation'");\n'
                    sedMultiLine "$file" "$find" "$replace"
                done
            done
            # Add references to attributes
            # ...
#######################################################################################

            # Add the component to the .h file

        # If it's a service
        elif [ "$ascIsAService" != "" ]
        then
            # Find the service files
            cd "$ascToLinkPath"
            serviceFiles=( $(ls src) )
            # For each file, find its service and operations
            for serviceFile in ${serviceFiles[@]}
            do
                cd "$ascToLinkPath"
                serviceName=$(grep 'Service(' src/$serviceFile | sed "s|.*(\"||; s|\".*||")
                # Add the service to the start script
                cd "$newAcPath"
                file='start.ops'
                find='# Set up service plugins'
                replace='# Set up service plugins\nrequire("'$serviceName'")\nloadService("controller", "'$serviceName'")\n'
                sedMultiLine "$file" "$find" "$replace"

                # Find the operations for this file
                cd "$ascToLinkPath"
                operationNames=( $(grep 'addOperation(' src/$serviceFile | sed "s|.*addOperation(\"||; s|\".*||") )
                for operation in ${operationNames[@]}
                do
                    # Find the operation's returnVar and arguments
                    cd "$ascToLinkPath"
                    callbackFunction=$(grep "addOperation(\"$operation" src/$serviceFile | sed 's|.*::||; s|,.*||')
                    args=$(grep "::${callbackFunction}(" src/$serviceFile | sed "s|.*(||; s|).*||; s|\(.* \).*|\1|; s| .*,|,|g; s| $||")
                    returnVar=$(grep "::${callbackFunction}(" src/$serviceFile | sed "s| .*||")

                    # Add the operation to the .ccp file
                    cd "$newAcPath"
                    file='src/controller_component.cpp'
                    find='// Service plugins'
                    replace='// Service plugins\n    '$operation' = this->provides("'$serviceName'")->getOperation("'$operation'");'
                    sedMultiLine "$file" "$find" "$replace"

                    # Add the operation to the .h file
                    file="include/${newAcName}/controller_component.h"
                    find='// Service plugins'
                    replace='// Service plugins\n    OperationCaller<'$returnVar'('"$args"')> '$operation';'
                    sedMultiLine "$file" "$find" "$replace"
                done
            done
        else
            echo "ERROR: subcontroller ${ascToLinkName} not recognized as a service plugin or component"
        fi

    done

    # Do we want a gui output port?
    if [ "$guiOutDecision" = "n" ]
    then
        cd "$newAcPath"
        file='start.ops'
        lines=( "gui_policy.*status" "stream.*gui_data_out" )
        grepRemoveLines "$file" "${lines[@]}"

        file='src/controller_component.cpp'
        lines=( "guiDataOut" "Send data to the GUI" "readyToSend" )
        grepRemoveLines "$file" "${lines[@]}"

        file="include/${newAcName}/controller_component.h"
        lines=( "guiOut" "guiDataOut" )
        grepRemoveLines "$file" "${lines[@]}"

        rm msg/controller_log_data.msg
    fi


# For a sub-controller plugin
elif [[ -n $newAscPluginName ]]
then
    #cp -r ${templatesPath}/asc_service_plugin ${currentPath}/${newAscPluginName}
    echo "Dummy command"


# For a sub-controller component
elif [[ -n $newAscComponentName ]]
then
    echo "Dummy command"
    #cp -r ${templatesPath}/asc_component ${currentPath}/${newAscComponentName}
else
    echo "No controller name specified"
    exit 1
fi


echo "Controller generated"
exit 0
