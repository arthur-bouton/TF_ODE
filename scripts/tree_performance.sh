#!/bin/bash

if [ $# -ge 1 ]; then
	data_dir=$1
else
	>&2 echo Please specify the directory containing the parameters of each tree.
	exit 1
fi

csv_file=performance.csv

angle_list=$( seq -1 1 1 )
offset_list=$( seq -0.25 0.01 0.24 )

exe_file=../build/scene_1_mt
#exe_file=../build/scene_1_mt_inter

# Position of the trial duration in the string returned from the evaluation:
duration_pos=2


echo max_depth_1,max_depth_2,L1,success_percentage,average_time >> $data_dir/$csv_file

trap 'echo;exit' INT
for tree_dir in $data_dir/* ; do
	if ! [[ -f $tree_dir/params_1.yaml && -f $tree_dir/params_2.yaml ]]; then
		continue
	fi

	echo -e '\n\n===' $tree_dir '===\n'

	nsuccesses=0
	ntrials=0
	average_time=0
	for angle in $angle_list ; do
		for offset in $offset_list ; do
			result=($( $exe_file nodisplay $angle $tree_dir/params_ $offset 2> /dev/null ))
			echo ${result[*]}

			ntrials=$(( ntrials + 1 ))
			if [[ ${result[*]} == *'[Success]'* ]]; then
				nsuccesses=$(( nsuccesses + 1 ))
				average_time=$( bc -l <<< "$average_time + ${result[$duration_pos]}" )
			fi
		done
	done

	success_percentage=0
	if [ $nsuccesses -ne 0 ] ; then
		success_percentage=$( echo $nsuccesses $ntrials | awk '{printf "%.1f", $1/$2*100}' )
		average_time=$( echo $average_time $nsuccesses | awk '{printf "%.2f", $1/$2}' )
	fi

	max_depth_1=$( grep -oP '(?<=max_depth_1:).*?(?=,)' <<< $tree_dir )
	max_depth_2=$( grep -oP '(?<=max_depth_2:).*?(?=,)' <<< $tree_dir )
	L1=$( grep -oP '(?<=L1:).*?(?=})' <<< $tree_dir )

	echo $max_depth_1,$max_depth_2,$L1,$success_percentage,$average_time >> $data_dir/$csv_file
done
