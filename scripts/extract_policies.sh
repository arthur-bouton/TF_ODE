#!/bin/bash

data_dir=../training_data
dataset_name=rover_training_1
#watched_file=$data_dir/$dataset_name.data-00000-of-00001
watched_file=$data_dir/$dataset_name.txt
time_pos=14
time_max=18
extraction_dir=$data_dir/run_1/selected
n_digits=4

file_list+=" checkpoint"
file_list+=" $dataset_name.data-00000-of-00001"
file_list+=" $dataset_name.index"
file_list+=" $dataset_name.meta"


get_preffix_and_suffix()
{
	if [[ $1 == *'.'* ]]; then
		preffix=${1%.*}_
		suffix=.${1##*.}
	else
		preffix=$1'_'
		suffix=
	fi
}


ref_file=($file_list)
ref_file=${ref_file[0]}
get_preffix_and_suffix $ref_file
ref_preffix=$preffix
ref_suffix=$suffix
for (( i = 0 ; i < n_digits ; i++ )); do
	digit_expr+=[0-9]
done


get_file_number()
{
	file_number=$(ls $extraction_dir/$ref_preffix$digit_expr$ref_suffix 2> /dev/null)

	if [ $? -ne 0 ]; then
		printf -v file_number "%0${n_digits}i" 1
	else
		file_number=${file_number: -${#ref_suffix}-$n_digits:$n_digits}
		file_number=$(( 10#$file_number + 1 ))
		printf -v file_number "%0${n_digits}i" $file_number
	fi
}


extract_data()
{
	get_file_number

	mkdir -p $extraction_dir
	for file in $file_list; do
		get_preffix_and_suffix $file
		cp $data_dir/$file $extraction_dir/$preffix$file_number$suffix
	done
	echo "$file_number -- ${line[*]}" >> $extraction_dir/$dataset_name'_index.txt'
}


# Synchronous updates:
while inotifywait -qqe modify $watched_file; do
	line=($( tail -1 $watched_file ))
	if [[ ${line[*]} == *'[Success]'* ]]; then
		if (( $( echo "${line[$time_pos]} < $time_max" | bc -l ) )); then
			extract_data
			echo "Dataset $file_number extracted (time: ${line[$time_pos]})"
		fi
	fi
done


# Asynchronous updates:
#inotifywait -q -m -e modify $watched_file |
#while read -r filename event; do
	#line=($( tail -1 $watched_file ))
	#if [[ ${line[*]} == *'[Success]'* ]]; then
		#if (( $( echo "${line[$time_pos]} < $time_max" | bc -l ) )); then
			#extract_data
			#echo "Dataset $file_number extracted (time: ${line[$time_pos]})"
		#fi
	#fi
#done
