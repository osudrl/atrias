#!/bin/bash

# This code makes a few assumptions:
# 1) top-level controllers are prefixed with atc_
# 2) sub-level controllers are prefixed with asc_
# 3) sub-level controllers are located in atrias_controllers
# 4) there is only one orocos component per subcontroller package

function help
{
    echo
    echo "Usage: rosrun atrias controller-generation.sh"
    echo "This script creates controllers for use with atrias in the current directory."
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

function sedOrdered
{
    # This function is to keep the generated code readable by ordering lines from 0..1..2..etc instead of ..2..1..0.
    # This function relies on these variables being passed implicitly:
    # unique, prevUnique, uniqueNumber, prevUniqueNumber
    file="$1"
    find="$2"
    replace="$3"
    if [ "$uniqueNumber" -gt "0" ]
    then
        find=$(echo "$replace" | sed "s|^...||; s|${unique}|${prevUnique}|g; s|${uniqueNumber}|${prevUniqueNumber}|g")
    fi
    sedMultiLine "$file" "$find" "$replace"
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

function determineSubcontrollers
{
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
        templateAcName='atc_component'
        newCppFile="controller_component.cpp"
        newHFile="controller_component.h"
        templateCppFile="controller_component.cpp"
        templateHFile="controller_component.h"
        files=( start.ops manifest.xml mainpage.dox CMakeLists.txt src/${newCppFile} src/controller_gui.cpp include/${newAtcName}/${newHFile} include/${newAtcName}/controller_gui.h )

        echo "Do you want this controller to have an output port to the gui? (y/n)"
        read guiOutDecision
        while [[ "$guiOutDecision" != "y" && "$guiOutDecision" != "n" ]]
        do
            echo "Unrecognized answer.  y/n?"
            read guiOutDecision
        done

        # Figure out if the user wants subcontrollers
        determineSubcontrollers

        break
        ;;
    2)
        echo "What do you want to name it? (e.g. asc_example_name)"
        read newAscPluginName
        templateAcName='asc_service_plugin'
        newCppFile="${newAscPluginName}-service.cpp"
        newHFile="${newAscPluginName}-service.h"
        templateCppFile="asc_service_plugin-service.cpp"
        templateHFile="asc_service_plugin-service.h"
        files=( manifest.xml CMakeLists.txt src/${newCppFile} include/${newAscPluginName}/${newHFile} )
        break
        ;;
    3)
        echo "What do you want to name it? (e.g. asc_example_name)"
        read newAscComponentName
        templateAcName='asc_component'
        newCppFile="controller_component.cpp"
        newHFile="controller_component.h"
        templateCppFile="controller_component.cpp"
        templateHFile="controller_component.h"
        files=( start.ops manifest.xml mainpage.dox CMakeLists.txt src/${newCppFile} include/${newAscComponentName}/${newHFile} )

        # Figure out if the user wants subcontrollers
        determineSubcontrollers

        break
        ;;
    *)
        echo "Not a valid selection.  Please select 1, 2, or 3."
        ;;
    esac
done

# Save the name
newAcName="${newAscPluginName}${newAscComponentName}${newAtcName}"

echo "Give a description of this controller"
read description

# Sanity check: Does this controller already exist in this directory?
if [ -d "$newAcName" ]
then
    echo "ERROR: This controller already exists.  Manually remove the existing one to continue."
    exit 1
fi
# Sanity check: Did we get a valid name?
if [ "$newAcName" = "" ]
then
    echo "ERROR: No controller name specified."
    exit 1
fi

# Double-check this is what we want to do
clear
echo "This script will create the package $newAcName in this directory with these subcontrollers:"
echo "${ascsToLink[@]-None}" | sed "s| |\n|g"
echo
echo "If this is what you want to do, press Enter..."
read



################# Generate the files #################
# Names
newAcCamelName=$(echo $newAcName | sed "s|\(^...\)|\U\1|; s|_\(.\)|\U\1|g")
templateAcCamelName=$(echo $templateAcName | sed "s|\(^...\)|\U\1|; s|_\(.\)|\U\1|g")
newAcIncludeGuard=$(echo ${newAcName} | sed 's|\(.*\)|__\U\1_H__|; s|-|_|g')
templateAcIncludeGuard=$(echo ${templateAcName} | sed 's|\(.*\)|__\U\1_H__|; s|-|_|g')
# Paths
newAcPath="${currentPath}/${newAcName}"
templateAcPath="${templatesPath}/${templateAcName}"

# Copy the template and change to the new directory
cp -r "$templateAcPath" "$newAcPath"
# Sanity check (This directory must exist)
if [ ! -d $newAcPath ]
then
    echo "ERROR: Could not copy from the templates directory"
    exit 1
fi
cd "$newAcPath"
# Move the include folder to its appropriate location
mv "include/$templateAcName" "include/$newAcName"
mv "include/${newAcName}/${templateHFile}" "include/${newAcName}/${newHFile}" &> /dev/null
mv "src/$templateCppFile" "src/$newCppFile" &> /dev/null
# Replace the include guard
sed -i "s|${templateAcIncludeGuard}|${newAcIncludeGuard}|" "include/${newAcName}/${newHFile}"
# Rename things
for file in ${files[@]}
do
    sed -i "s|${templateAcName}|${newAcName}|g; s|${templateAcCamelName}|${newAcCamelName}|g" $file
done
# Controller description
echo "description=$description" > controller.txt

# If this is a plugin, exit here
if [ "$newAscPluginName" != "" ]
then
    echo "Controller generated"
    exit 0
fi

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
        prevUniqueNumber=$(( $uniqueNumber - 1 ))
        unique=${uniqueNamePrefix}${uniqueNumber}
        prevUnique=${uniqueNamePrefix}${prevUniqueNumber}
    done

    # Add the subcontroller as a dependency
    if [ "$uniqueNumber" -eq "0" ]
    then
        cd "${newAcPath}"
        file='manifest.xml'
        find='\n</package>'
        replace='  <depend package="'$ascToLinkName'" />\n&'
        sedMultiLine "$file" "$find" "$replace"
    fi

    # If it's a component
    if [ "$ascIsAComponent" != "" ]
    then
        foundAComponent=1
        # Get the component name
        cd "$ascToLinkPath"
        componentName=$(grep "ORO_CREATE_COMPONENT" src/$newCppFile | sed 's|ORO_CREATE_COMPONENT(||; s|).*||')

        # Add the component to the start script
        cd "${newAcPath}"
        file='start.ops'
        if [ "$uniqueNumber" -eq "0" ]
        then
            # Set up subcontrollers
            find='# Set up subcontrollers'
            replace='&\nimport("'$ascToLinkName'")\nvar string '${unique}Name' = atrias_cm.getUniqueName("controller", "'${uniqueNamePrefix}'")\nloadComponent('${unique}Name', "'$componentName'")\n'
            sedMultiLine "$file" "$find" "$replace"

        else
            # Set up subcontrollers
            find='loadComponent('${prevUnique}Name', "'$componentName'")'
            replace='&\nvar string '${unique}Name' = atrias_cm.getUniqueName("controller", "'${uniqueNamePrefix}'")\nloadComponent('${unique}Name', "'$componentName'")'
            sedMultiLine "$file" "$find" "$replace"

        fi

        # Subcontroller names
        find='# Pass the names of the subcontrollers to the controller'
        replace='&\ncontroller.'${unique}Name' = '${unique}Name''
        sedOrdered "$file" "$find" "$replace"

        # Connect to subcontrollers
        find='# Connect this controller with its subcontrollers'
        replace='&\naddPeer("controller", '${unique}Name')'
        sedOrdered "$file" "$find" "$replace"

        # Add the component to the stop script
        file='stop.ops'
        find='controller.stop()'
        replace='&\nunloadComponent(controller.'${unique}Name')'
        sedOrdered "$file" "$find" "$replace"

        # Add the component to the .cpp and .h file
        file="src/${newCppFile}"
        find='// Add properties'
        replace='&\n    this->addProperty("'${unique}Name'", '${unique}Name');'
        sedOrdered "$file" "$find" "$replace"

        file="include/${newAcName}/${newHFile}"
        find='// Subcontroller names'
        replace='&\n    std::string '${unique}Name';'
        sedOrdered "$file" "$find" "$replace"

        find='// Subcontroller components'
        replace='&\n    TaskContext \*'$unique';'
        sedOrdered "$file" "$find" "$replace"


        # Operations
        cd "$ascToLinkPath"
        services=( $(grep 'this->provides(' src/${newCppFile} | sed "s|.*(\"||; s|\".*||") )
        for service in ${services[@]}
        do
            cd "$ascToLinkPath"
            operations=( $(grep '\->addOperation("' src/${newCppFile} | sed "s|.*(\"||; s|\".*||") )
            for operation in ${operations[@]}
            do
                cd "$ascToLinkPath"
                callbackFunction=$(grep "addOperation(\"$operation" src/${newCppFile} | sed 's|.*::||; s|,.*||')
                args=$(grep "::${callbackFunction}(" src/${newCppFile} | sed "s|.*(||; s|).*||; s| [[:alnum:]]*$||; s| [[:alnum:]]*,|,|g")
                returnVar=$(grep "::${callbackFunction}(" src/${newCppFile} | sed "s| .*||")
                operationReference=$(echo "${unique}_${operation}" | sed "s|_\(.\)|\U\1|g")
                prevOperationReference=$(echo "${prevUnique}_${operation}" | sed "s|_\(.\)|\U\1|g")

                cd "$newAcPath"
                file="include/${newAcName}/${newHFile}"
                find='// Subcontroller operations'
                replace='&\n    OperationCaller<'$returnVar'('"$args"')> '${operationReference}';'
                sedOrdered "$file" "$find" "$replace"

                file="src/${newCppFile}"
                find='// Connect to the subcontrollers'
                replace='&\n    '$unique' = this->getPeer('${unique}Name');\n    if ('$unique')\n        '${operationReference}' = '$unique'->provides("'$service'")->getOperation("'$operation'");\n'
                sedOrdered "$file" "$find" "$replace"

            done
        done

        # Properties
        cd "$ascToLinkPath"
        properties=( $(grep 'this->addProperty("' src/${newCppFile} | sed "s|.*(\"||; s|\".*||") )
        for property in ${properties[@]}
        do
            cd "$ascToLinkPath"
            propType=$(grep " ${property},\| ${property};" include/${ascToLinkName}/${newHFile} | sed "s|^\s*||; s| .*||")

            cd "$newAcPath"
            file="include/${newAcName}/${newHFile}"
            find='// Service properties'
            replace='&\n    Property<'$propType'> '${property}${uniqueNumber}';'
            sedOrdered "$file" "$find" "$replace"

            file="src/${newCppFile}"
            find='// Get references to subcontroller component properties'
            replace='&\n    '${property}${uniqueNumber}' = '${unique}'->properties()->getProperty("'${property}'");'
            sedOrdered "$file" "$find" "$replace"
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
                file="src/${newCppFile}"
                find='// Service plugins'
                replace='&\n    '$operation' = this->provides("'$serviceName'")->getOperation("'$operation'");'
                sedMultiLine "$file" "$find" "$replace"

                # Add the operation to the .h file
                file="include/${newAcName}/${newHFile}"
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

    file="src/${newCppFile}"
    lines=( "Let the GUI know the controller run state" "guiDataOut" "guiOut" "Send data to the GUI" "readyToSend" )
    grepRemoveLines "$file" "${lines[@]}"

    file="include/${newAcName}/${newHFile}"
    lines=( "guiOut" "guiDataOut" "#include <atc_test/controller_status.h>" )
    grepRemoveLines "$file" "${lines[@]}"
fi

# Clean up unused flags
# If no service plugin
if [[ -z "$foundAService" ]]
then
    cd "$newAcPath"
    file='start.ops'
    lines=( "# Set up service plugins" )
    grepRemoveLines "$file" "${lines[@]}"
    file="src/${newCppFile}"
    lines=( "// Service plugins" )
    grepRemoveLines "$file" "${lines[@]}"
fi
# If no component
if [[ -z "$foundAComponent" ]]
then
    cd "$newAcPath"
    file='start.ops'
    lines=( "# Set up subcontrollers" "# Connect this controller with its subcontrollers" "# Pass the names of the subcontrollers to the controller" )
    grepRemoveLines "$file" "${lines[@]}"
    file="src/${newCppFile}"
    lines=( "// Add properties" "// Connect to the subcontrollers" "// Get references to subcontroller component properties" )
    grepRemoveLines "$file" "${lines[@]}"
    file="include/${newAcName}/${newHFile}"
    lines=( "// Subcontroller names" "// Subcontroller components" "// Service properties" )
    grepRemoveLines "$file" "${lines[@]}"
fi
# If no service plugin or component
if [[ -z "$foundAService" && -z "$foundAComponent" ]]
then
    cd "$newAcPath"
    file="include/${newAcName}/${newHFile}"
    lines=( "// Subcontroller operations" )
    grepRemoveLines "$file" "${lines[@]}"

    file="src/${newCppFile}"
    find='::configureHook() {[[:space:]]*'
    replace='::configureHook() {\n    '
    sedMultiLine "$file" "$find" "$replace"
fi

# Clean up excessive newlines
for i in "start.ops" "src/${newCppFile}" "include/${newAcName}/${newHFile}"
do
    file="$i"
    find='\n\{3,\}' # Match at least three newlines
    replace='\n\n'
    sedMultiLine "$file" "$find" "$replace"
done

echo "Controller generated"
exit 0
