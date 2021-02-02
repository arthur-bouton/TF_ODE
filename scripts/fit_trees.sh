#!/bin/bash

if [ $# -ge 1 ]; then
	log_dir=$1
else
	>&2 echo Please specify the directory where to store the parameters of each tree.
	exit 1
fi
shift

if [ $# -ge 1 ]; then
	sample_files=( $* )
else
	>&2 echo Please specify the all the sample files.
	exit 1
fi



option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 0 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 0 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 0.05 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 0.05 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 0.1 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 0.2 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 0.2 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 0.5 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 0.5 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 1 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 1 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 2 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 2 };"
option_list+="break;"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 0, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 0, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 1, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 1, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 2, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 2, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 3, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 3, L1: 3 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 4, L1: 3 };"




IFS=';'
trap 'echo;exit' INT
prev_tree_params=-
for options in $option_list ; do
	if [[ $options == break ]] ; then
		prev_tree_params=-
		continue
	fi
	echo -e '\n\n===' $options '===\n'
	tree_dir=$log_dir/$(echo $options | tr -d ' ')
	mkdir -p $tree_dir
	tree_params=$tree_dir/params_
	python -u policy_tree.py "$options" $tree_params $prev_tree_params ${sample_files[@]} | tee $tree_dir/log.txt
	prev_tree_params=$tree_params
done


echo oblique,\
     max_depth_1,\
	 max_depth_2,\
	 min_samples,\
	 loss_tol,\
	 L1,\
	 abs_err_1,\
	 abs_err_2,\
	 quad_err_1,\
	 quad_err_2,\
	 nz_params_1,\
	 t_params_1,\
	 nz_params_2,\
	 t_params_2 >> $log_dir/tree_infos.csv

for tree_dir in $log_dir/* ; do
	csv_line=$(tail -1 $tree_dir/log.txt)
	if [ $(echo $csv_line | tr -d -c , | wc -c) -eq 13 ] ; then
		echo $csv_line >> $log_dir/tree_infos.csv
	else
		1>&2 echo
		1>&2 echo Last line in $tree_dir/log.txt does not meet the CSV requirements:
		1>&2 echo '->' $csv_line
	fi
done
