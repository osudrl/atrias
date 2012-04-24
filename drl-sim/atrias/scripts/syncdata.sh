#!/bin/bash

# Sync all *.log files from zotac-3 (on robot) to local log files directory.
# This does not delete any files from the local directory even if the files are
# deleted on zotac-3.

if [ $HOSTNAME = "drl-sim" ]; then
    rsync -avz -e ssh drl@zotac-3:atrias/drl-sim/atrias/log_files/*.log `rospack find atrias`/log_files
else
    # TODO: This script should be able to work from anywhere.
    echo "This script should be run from the desktop!"
fi

