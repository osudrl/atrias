#!/bin/sh

# This script loads "kpaecsboard_mod.ko" and dynamically creates needed device files

#!/bin/sh
module="kpaecsboard_mod"
module_path="lib"
device="kpaecs"
mode="664"

# Numbers of devices to create:
device_numbers="0 1 2 3 4 5"

#-----------------------------------------------------------------------------
func_usage ()
{
    cat << EOF

This script is for loading/unloading KPA EtherCAT slave
board driver (kernel-space module).

Usage:
    $0 { load | unload | help }

Examples:
    $0 load
    $0 unload
    $0 help

EOF
}

#-----------------------------------------------------------------------------
func_load()
{
    # Group: since distributions do it differently, look for wheel or use staff
    if grep '^staff:' /etc/group > /dev/null; then
        group="staff"
    else
        group="wheel"
    fi

    /sbin/insmod -f ${module_path}/$module.ko $* || exit 1

    # Find out major number dynamically assigned to our character device driver
    major=`cat /proc/devices | awk "\\$2==\"$device\" {print \\$1}"`
    echo "major=${major}"

    # Remove stale nodes and replace them, then give gid and perms

    for i in ${device_numbers}; do \
        rm -f /dev/${device}$i; \
        mknod /dev/${device}$i c $major $i; \
        chgrp $group /dev/${device}$i; \
        chmod $mode  /dev/${device}$i; \
    done
}

#-----------------------------------------------------------------------------
func_unload()
{
    # invoke rmmod with all arguments we got
    /sbin/rmmod $module $* || exit 1

    # Remove stale nodes
    for i in ${device_numbers}; do \
        rm -f /dev/${device}$i; \
    done
}

#-----------------------------------------------------------------------------
case $1 in
    help|h)
        func_usage
        ;;

    load|l)
        func_load
        ;;

    unload|u)
        func_unload
        ;;

    *)
        func_usage
        exit 1
        ;;
esac
