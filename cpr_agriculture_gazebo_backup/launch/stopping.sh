#!/bin/bash

exit_status=0

_term() {
	exit_status=$?
	kill -INT "$child" 2>/dev/null
}

trap _term SIGINT
vel = 0.1
mu = 1
world= control_world.world

while [ $vel -le 20 ]
do

roslaunch cpr_agriculture_gazebo stopping_data.launch \
	world:=$world vel:=$vel mu:=$mu gui:=false &

vel=$(( $vel + 0.1 ))

child=$!
wait "$child"

if [ $exit_status -eq 130 ]; then 
	break
fi

done

