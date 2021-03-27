#!/bin/bash

exit_status=0

_term() {
	exit_status=$?
	kill -INT "$child" 2>/dev/null
}

trap _term SIGINT
run_num=991
angle=0



world=1.5m_ramp_up_world_0.65_angle.world

while [ $run_num -le 1000 ]
do

roslaunch cpr_agriculture_gazebo ramp_up.launch \
	world:=$world run_num:=$run_num angle:=$angle gui:=false &

run_num=$(( $run_num + 1 ))

child=$!
wait "$child"

if [ $exit_status -eq 130 ]; then 
	break
fi

done

