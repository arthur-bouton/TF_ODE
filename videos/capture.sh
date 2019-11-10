#!/bin/bash

if [ "$1" == 'rock_step_3' ]; then
	../build/rover_training_1_exe capture ../training_data/run_9/selected/rover_training_1_0027 > ../videos/data/rock_step_3.dat

elif [ "$1" == 'rock_step_3_bis' ]; then
	../build/rover_training_1_exe capture ../training_data/run_9/selected2/rover_training_1_0001 > ../videos/data/rock_step_3_bis.dat

elif [ "$1" == 'groove' ]; then
	../build/rover_training_1_exe capture ../training_data/run_10/selected2/rover_training_1_0004 > ../videos/data/groove.dat

fi
