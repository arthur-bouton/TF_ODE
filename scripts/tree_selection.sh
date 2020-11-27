#!/bin/bash

if [ $# -ge 1 ]; then
	log_dir=$1
else
	>&2 echo Please specify the directory where to store the parameters of each tree.
	exit 1
fi



#option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1: 0 };"
#option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1: 0.1 };"
#option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1: 0.5 };"
#option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1: 1 };"
#option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1: 2 };"
#option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1: 3 };"


#option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1: 0 };"
#option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1: 0.1 };"
#option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1: 0.5 };"
#option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1: 1 };"
#option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1: 2 };"
#option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1: 3 };"


#option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1: 0 };"
#option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1: 0.1 };"
#option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1: 0.5 };"
#option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1: 1 };"
#option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1: 2 };"
#option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1: 3 };"



option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 1, L1: 0 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 1, L1: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 1, L1: 0.5 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 1, L1: 1 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 1, L1: 2 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 1, L1: 3 };"


option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 2, L1: 0 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 2, L1: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 2, L1: 0.5 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 2, L1: 1 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 2, L1: 2 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 2, L1: 3 };"


option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 3, L1: 0 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 3, L1: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 3, L1: 0.5 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 3, L1: 1 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 3, L1: 2 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 3, L1: 3 };"



IFS=';'
trap 'echo;exit' INT
for options in $option_list ; do
	echo -e '\n\n===' $options '===\n'
	tree_dir=$log_dir/$(echo $options | tr -d ' ')
	mkdir -p $tree_dir
	python -u policy_tree.py "$options" $tree_dir/params_ |& tee $tree_dir/log.txt
done


for tree_dir in $log_dir/* ; do
	csv_line=$(tail -1 $tree_dir/log.txt)
	if [ $(echo $csv_line | tr -d -c , | wc -c) -eq 13 ] ; then
		echo $csv_line >> tree_selection_mu05_inter.csv
	else
		1>&2 echo
		1>&2 echo Last line in $tree_dir/log.txt does not meet the CSV requirements:
		1>&2 echo '->' $csv_line
	fi
done
