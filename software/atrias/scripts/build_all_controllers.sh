#!/bin/bash
cd `rospack find atrias`/../atrias_controllers
ls -d -1 */ | sed "s/\///g" | sed ':a;N;$!ba;s/\n/ /g' | rosmake --build-everything
