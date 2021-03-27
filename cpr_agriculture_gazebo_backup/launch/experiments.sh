#!/bin/bash

exit_status=0

_term() {
	exit_status=$?
	kill -INT "$child" 2>/dev/null
}

trap _term SIGINT
run_num=961
angle=0
thresh=2
world=control_env.world



while [ $run_num -le 962 ]
do

roslaunch cpr_agriculture_gazebo training_env.launch &

roslaunch cpr_agriculture_gazebo training.launch &

roslaunch cpr_agriculture_gazebo reset.launch &

run_num=$(( $run_num + 1 ))

child=$!
wait "$child"

if [ $exit_status -eq 130 ]; then 
	break
fi

done
