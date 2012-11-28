#!/bin/bash

ROBOT="i1000a-1"
GUI_COMPUTER="cov022-5"

if [ $HOSTNAME == $GUI_COMPUTER ]; then
    rsync -avz --progress -e ssh drl@$ROBOT:/home/drl/atrias/software/atrias/bagfiles/*.bag `rospack find atrias`/bagfiles
elif [ $HOSTNAME == $ROBOT ]; then
    rsync -avz --progress `rospack find atrias`/bagfiles/*.bag -e ssh drl@$GUI_COMPUTER:/home/drl/atrias/software/atrias/bagfiles
else
    echo "Run this from either the GUI computer or the robot!"
fi

exit 0

