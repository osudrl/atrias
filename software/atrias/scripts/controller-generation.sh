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
    sed -n '1h;1!H;${;g;s|'"$find"'|'"$replace"'|g;p;}' "${file}.old" > "$file"
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
        break
        ;;
    3)
        echo "What do you want to name it? (e.g. asc_example_name)"
        read newAscComponentName
        break
        ;;
    *)
        echo "Not a valid selection.  Please select 1, 2, or 3."
        ;;
    esac
done

echo "Give a description of this controller"
read description

# Sanity check (Does this controller already exist in this directory?)
if [ -d "${newAscPluginName}${newAscComponentName}${newAtcName}" ]
then
    echo "ERROR: This controller already exists.  Manually remove the existing one to continue."
    exit 1
fi

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
        echo "${ascList[@]}" | sed "s| |\n|g" | nl -w1 -s") "
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
    newAcIncludeGuard=$(echo $newAcName | sed 's|\(.*\)|__\U\1__|')
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
    # Replace the include guard
    sed -i "s|__ATC_COMPONENT__|${newAcIncludeGuard}|" "include/${newAcName}/controller_component.h"

    # Rename things
    files=( start.ops manifest.xml mainpage.dox CMakeLists.txt src/controller_component.cpp src/controller_gui.cpp include/${newAcName}/controller_component.h include/${newAcName}/controller_gui.h )
    for file in ${files[@]}
    do
        sed -i "s|atc_component|${newAcName}|g; s|ATCComponent|${newAcCamelName}|g" $file
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

        # Add the subcontroller as a dependency
        if [ "$uniqueNumber" -eq "0" ]
        then
            cd "${newAcPath}"
            file='manifest.xml'
            find='\n</package>'
            replace='  <depend package="'$ascToLinkName'" />\n&'
            sedMultiLine "$file" "$find" "$replace"
            echo "Linked a dependency!"
        fi

        # If it's a component
        if [ "$ascIsAComponent" != "" ]
        then
            foundAComponent=1
            # Get the component name
            cd "$ascToLinkPath"
            componentName=$(grep "ORO_CREATE_COMPONENT" src/controller_component.cpp | sed 's|ORO_CREATE_COMPONENT(||; s|).*||')

            # Add the component to the start script
            cd "${newAcPath}"
            file='start.ops'
            if [ "$uniqueNumber" -eq "0" ]
            then
                find='# Set up subcontrollers'
                replace='&\nimport("'$ascToLinkName'")\nvar string '${unique}Name' = atrias_cm.getUniqueName("controller", "'${uniqueNamePrefix}'")\nloadComponent('${unique}Name', "'$componentName'")\n'
            else
                find='import("'$ascToLinkName'")'
                replace='&\nvar string '${unique}Name' = atrias_cm.getUniqueName("controller", "'${uniqueNamePrefix}'")\nloadComponent('${unique}Name', "'$componentName'")'
            fi
            sedMultiLine "$file" "$find" "$replace"

            find='# Connect this controller with its subcontrollers'
            replace='&\naddPeer("controller", '${unique}Name')'
            sedMultiLine "$file" "$find" "$replace"

            find='# Pass the names of the subcontrollers to the controller'
            replace='&\ncontroller.'${unique}Name' = '${unique}Name''
            sedMultiLine "$file" "$find" "$replace"

            # Add the component to the stop script
            file='stop.ops'
            find='controller.stop()'
            replace='&\nunloadComponent(controller.'${unique}Name')'
            sedMultiLine "$file" "$find" "$replace"

            # Add the component to the .cpp and .h file
            file='src/controller_component.cpp'
            find='// Add properties'
            replace='&\n    this->addProperty("'${unique}Name'", '${unique}Name');'
            sedMultiLine "$file" "$find" "$replace"

            file="include/${newAcName}/controller_component.h"
            find='// Subcontroller names'
            replace='&\n    std::string '${unique}Name';'
            sedMultiLine "$file" "$find" "$replace"

            find='// Subcontroller components'
            replace='&\n    TaskContext *'$unique';'
            sedMultiLine "$file" "$find" "$replace"

            # Operations
            cd "$ascToLinkPath"
            services=( $(grep 'this->provides(' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
            for service in ${services[@]}
            do
                cd "$ascToLinkPath"
                operations=( $(grep '\->addOperation("' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
                for operation in ${operations[@]}
                do
                    cd "$ascToLinkPath"
                    callbackFunction=$(grep "addOperation(\"$operation" src/controller_component.cpp | sed 's|.*::||; s|,.*||')
                    args=$(grep "::${callbackFunction}(" src/controller_component.cpp | sed "s|.*(||; s|).*||; s| [[:alnum:]]*$||; s| [[:alnum:]]*,|,|g")
                    returnVar=$(grep "::${callbackFunction}(" src/controller_component.cpp | sed "s| .*||")
                    operationReference=$(echo "${unique}_${operation}" | sed "s|_\(.\)|\U\1|g")

                    cd "$newAcPath"
                    file="include/${newAcName}/controller_component.h"
                    find='// Subcontroller operations'
                    replace='&\n    OperationCaller<'$returnVar'('"$args"')> '${operationReference}';'
                    sedMultiLine "$file" "$find" "$replace"

                    file='src/controller_component.cpp'
                    find='// Connect to the subcontrollers'
                    replace='&\n    '$unique' = this->getPeer('${unique}Name');\n    if ('$unique')\n        '${operationReference}' = '$unique'->provides("'$service'")->getOperation("'$operation'");\n'
                    sedMultiLine "$file" "$find" "$replace"
                done
            done
            # Properties
            cd "$ascToLinkPath"
            properties=( $(grep 'this->addProperty("' src/controller_component.cpp | sed "s|.*(\"||; s|\".*||") )
            for property in ${properties[@]}
            do
                cd "$ascToLinkPath"
                propType=$(grep " ${property},\| ${property};" include/${ascToLinkName}/controller_component.h | sed "s|^\s*||; s| .*||")

                cd "$newAcPath"
                file="include/${newAcName}/controller_component.h"
                find='// Service properties'
                replace='&\n    Property<'$propType'> '${property}${uniqueNumber}';'
                sedMultiLine "$file" "$find" "$replace"

                file='src/controller_component.cpp'
                find='// Get references to subcontroller component properties'
                replace='&\n    '${property}${uniqueNumber}' = '${unique}'->properties()->getProperty("'${property}'");'
                sedMultiLine "$file" "$find" "$replace"
            done


        # If it's a service
        elif [ "$ascIsAService" != "" ]
        then
            foundAService=1
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
                replace='&\nrequire("'$serviceName'")\nloadService("controller", "'$serviceName'")\n'
                sedMultiLine "$file" "$find" "$replace"

                # Find the operations for this file
                cd "$ascToLinkPath"
                operationNames=( $(grep 'addOperation(' src/$serviceFile | sed "s|.*addOperation(\"||; s|\".*||") )
                for operation in ${operationNames[@]}
                do
                    # Find the operation's returnVar and arguments
                    cd "$ascToLinkPath"
                    callbackFunction=$(grep "addOperation(\"$operation" src/$serviceFile | sed 's|.*::||; s|,.*||')
                    args=$(grep "::${callbackFunction}(" src/$serviceFile | sed "s|.*(||; s|).*||; s| [[:alnum:]]*$||; s| [[:alnum:]]*,|,|g")
                    returnVar=$(grep "::${callbackFunction}(" src/$serviceFile | sed "s| .*||")

                    # Add the operation to the .ccp file
                    cd "$newAcPath"
                    file='src/controller_component.cpp'
                    find='// Service plugins'
                    replace='&\n    '$operation' = this->provides("'$serviceName'")->getOperation("'$operation'");'
                    sedMultiLine "$file" "$find" "$replace"

                    # Add the operation to the .h file
                    file="include/${newAcName}/controller_component.h"
                    find='// Subcontroller operations'
                    replace='&\n    OperationCaller<'$returnVar'('"$args"')> '$operation';'
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

    # Clean up the flags that are unused
    if [[ -z "$foundAService" ]] # No service plugin
    then
        cd "$newAcPath"
        file='start.ops'
        lines=( "# Set up service plugins" )
        grepRemoveLines "$file" "${lines[@]}"
        file='src/controller_component.cpp'
        lines=( "// Service plugins" )
        grepRemoveLines "$file" "${lines[@]}"
    fi
    if [[ -z "$foundAComponent" ]] # No component
    then
        cd "$newAcPath"
        file='start.ops'
        lines=( "# Set up subcontrollers" "# Connect this controller with its subcontrollers" "# Pass the names of the subcontrollers to the controller" )
        grepRemoveLines "$file" "${lines[@]}"
        file='src/controller_component.cpp'
        lines=( "// Add properties" "// Connect to the subcontrollers" "// Get references to subcontroller component properties" )
        grepRemoveLines "$file" "${lines[@]}"
        file="include/${newAcName}/controller_component.h"
        lines=( "// Subcontroller names" "// Subcontroller components" "// Service properties" )
        grepRemoveLines "$file" "${lines[@]}"
    fi
    if [[ -z "$foundAService" && -z "$foundAComponent" ]]
    then
        cd "$newAcPath"
        file="include/${newAcName}/controller_component.h"
        lines=( "// Subcontroller operations" )
        grepRemoveLines "$file" "${lines[@]}"
    fi

    # Clean up excessive newlines
    for i in "start.ops" "src/controller_component.cpp" "include/${newAcName}/controller_component.h"
    do
        file="$i"
        find='\n\{3,\}' # Match at least three newlines
        replace='\n\n'
        sedMultiLine "$file" "$find" "$replace"
    done


# For a sub-controller plugin
elif [[ -n $newAscPluginName ]]
then
    cp -r ${templatesPath}/asc_service_plugin ${currentPath}/${newAscPluginName}


# For a sub-controller component
elif [[ -n $newAscComponentName ]]
then
    cp -r ${templatesPath}/asc_component ${currentPath}/${newAscComponentName}

else
    echo "No controller name specified"
    exit 1
fi


echo "Controller generated"
exit 0
