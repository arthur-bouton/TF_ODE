#!/bin/bash

angle_list=$( seq -2 1 2 )
even_offset_list=$( seq -0.25 0.02 0.23 )
odd_offset_list=$( seq -0.24 0.02 0.24 )

exe_file=../build/rover_training_1_exe
actor_dir=../training_data/Ry05t05c_eg87/picked/actor_02

mkdir -p ../training_data/samples
samples_id=../training_data/samples/samples_Ry05t05c_eg87_p02_mu05


for angle in $angle_list ; do
	for parity in even odd ; do
		sample_file=${samples_id}_angle${angle}_${parity}.dat
		offset_list=${parity}_offset_list
		for offset in ${!offset_list} ; do
			echo angle=$angle parity=$parity offset=$offset
			output=$( $exe_file eval $actor_dir $angle $offset 2> /dev/null )
			result=( $( echo "$output" | tail -1 ) )
			if [[ ${result[*]} == *'[Success]'* ]]; then
				echo New trial: angle=$angle offset=$offset >> $sample_file
				echo "$output" >> $sample_file
			else
				echo -e '\033[1;31m' Failed for angle=$angle and offset=$offset '\033[0;39m'
			fi
		done
	done
done
