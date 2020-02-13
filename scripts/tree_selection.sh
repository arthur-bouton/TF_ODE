#!/bin/bash

log_dir=tree_data

option_list+="{ oblique: false, max_depth_1: 0, max_depth_2: 0, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 0, max_depth_2: 0, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 0, max_depth_2: 0, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 0, max_depth_2: 0, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 1, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 1, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 1, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 1, max_depth_2: 1, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 2, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 2, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 2, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 2, max_depth_2: 2, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 3, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 3, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 3, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 3, max_depth_2: 3, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 4, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 4, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 4, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 4, max_depth_2: 4, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 5, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 5, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 5, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 5, max_depth_2: 5, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 6, max_depth_2: 6, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 6, max_depth_2: 6, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 6, max_depth_2: 6, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 6, max_depth_2: 6, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 7, max_depth_2: 7, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 7, max_depth_2: 7, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 7, max_depth_2: 7, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 7, max_depth_2: 7, L1_reg: 1 };"

option_list+="{ oblique: false, max_depth_1: 8, max_depth_2: 8, L1_reg: 0 };"
option_list+="{ oblique: false, max_depth_1: 8, max_depth_2: 8, L1_reg: 0.01 };"
option_list+="{ oblique: false, max_depth_1: 8, max_depth_2: 8, L1_reg: 0.1 };"
option_list+="{ oblique: false, max_depth_1: 8, max_depth_2: 8, L1_reg: 1 };"

option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1_reg: 0 };"
option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1_reg: 0.01 };"
option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1_reg: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 1, max_depth_2: 1, L1_reg: 1 };"

option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1_reg: 0 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1_reg: 0.01 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1_reg: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 2, max_depth_2: 2, L1_reg: 1 };"

option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1_reg: 0 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1_reg: 0.01 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1_reg: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 3, max_depth_2: 3, L1_reg: 1 };"

option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 4, L1_reg: 0 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 4, L1_reg: 0.01 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 4, L1_reg: 0.1 };"
option_list+="{ oblique: true, max_depth_1: 4, max_depth_2: 4, L1_reg: 1 };"

IFS=';'
trap 'echo;exit' INT
for options in $option_list ; do
	echo -e '\n\n===' $options '===\n'
	tree_dir=$log_dir/$(echo $options | tr -d ' ')
	mkdir -p $tree_dir
	python -u policy_tree.py "$options" $tree_dir/params_ |& tee $tree_dir/log.txt
done
