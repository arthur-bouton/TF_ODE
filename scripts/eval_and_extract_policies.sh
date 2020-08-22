#!/bin/bash

# Identifier name for the training data:
run_id=step_05_nobat_nosym_1

# Maximum duration of a successful run for it to be extracted:
target_duration=30

# Maximum amount of data to extract:
max_extractions=3

# Number of digits for the identifiers of extracted models:
n_digits=4

# Position of the trial duration in the string returned from the evaluation:
duration_pos=2

data_dir=../training_data/$run_id
dataset_name=rover_training_1
watched_file=$data_dir/$dataset_name'_log.txt'
extraction_dir=$data_dir/selected
tmp_storage_dir=/tmp/$dataset_name"_tmp_graph_data"

file_list+=" checkpoint"
file_list+=" $dataset_name.data-00000-of-00001"
file_list+=" $dataset_name.index"
file_list+=" $dataset_name.meta"

export TF_CPP_MIN_LOG_LEVEL=1


# Do not write in files, only print results in the terminal:
if [ "$1" == '--display-only' ] || [ "$1" == '-d' ]; then
	echo "-- Display only --"
	display_only=true
fi

# Skip evaluations:
if [[ $1 =~ ^[0-9]+$ ]]; then
	skip=$1
	skip_count=$skip
else
	skip=0
fi


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


# Initialize file name parts:
ref_file=($file_list)
ref_file=${ref_file[0]}
get_preffix_and_suffix $ref_file
ref_preffix=$preffix
ref_suffix=$suffix
for (( i = 0 ; i < n_digits ; i++ )); do
	digit_expr+=[0-9]
done

# Create the temporary storage directory:
mkdir -p $tmp_storage_dir


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
	if [ $((++c)) -le $max_extractions ]; then
		get_file_number
	fi

	mkdir -p $extraction_dir
	for file in $file_list; do
		get_preffix_and_suffix $file
		cp $tmp_storage_dir/$file $extraction_dir/$preffix$file_number$suffix
	done
	echo "$file_number -- ${log[*]} => ${result[*]}" >> $extraction_dir/$dataset_name'_index.txt'
}


# Initialize file_number:
get_file_number

# Synchronous updates:
while inotifywait -qqe modify $watched_file; do
	if [ $((++skip_count)) -ge $skip ]; then
		skip_count=0

		log=($( tail -1 $watched_file ))

		for file in $file_list; do
			cp $data_dir/$file $tmp_storage_dir
		done
		echo -ne "> Evaluating ${log[*]::2}...\r"
		result=($(../build/rover_training_1_exe eval $tmp_storage_dir/$dataset_name 2> /dev/null))
		stat="${log[*]} => ${result[*]}"

		if [ "$display_only" == true ]; then
			echo $stat
		else
			echo $stat | tee -a $data_dir/$dataset_name'_stat.txt'
			if [[ ${result[*]} == *'[Success]'* ]]; then
				if (( $( echo "${result[$duration_pos]} <= $target_duration" | bc -l ) )); then
					extract_data
					echo "Dataset $file_number extracted (time: ${result[$duration_pos]})"
				fi
			fi
		fi
	fi
done


# Asynchronous updates:
#inotifywait -q -m -e modify $watched_file |
#while read -r filename event; do
	#log=($( tail -1 $watched_file ))
	#echo -ne "> Evaluating ${log[*]::2}...\r"
	#result=($(../build/rover_training_1_exe eval $data_dir/$dataset_name))
	#stat="${log[*]} => ${result[*]}"
	#if [ "$display_only" == true ]; then
		#echo $stat
	#else
		#echo $stat | tee -a $data_dir/$dataset_name'_stat.txt'
		#if [[ ${result[*]} == *'[Success]'* ]]; then
			#if (( $( echo "${result[$duration_pos]} <= $target_duration" | bc -l ) )); then
				#extract_data
				#echo "Dataset $file_number extracted (time: ${result[$duration_pos]})"
			#fi
		#fi
	#fi
#done
