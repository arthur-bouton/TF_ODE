#!/bin/bash

# Identifier name for the training data:
session_id=Rdxyt05

# Maximum duration of a successful run for it to be extracted:
target_duration=40

# Maximum amount of data to extract:
max_extractions=10

# Number of digits for the identifiers of extracted models:
n_digits=4

# Position of the trial duration in the string returned from the evaluation:
duration_pos=2

session_dir=../training_data/$session_id
watched_file=$session_dir/training.log
model_dir=actor
extraction_dir=$session_dir/picked
tmp_storage_dir=/tmp/${session_id}_tmp_model

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


# Create the temporary storage directory:
mkdir -p $tmp_storage_dir


# Create the expression for the number of digits looked for:
for (( i = 0 ; i < n_digits ; i++ )); do
	digit_expr+=[0-9]
done

get_file_number()
{
	file_number=$(ls -d $extraction_dir/$model_dir'_'$digit_expr 2> /dev/null)

	if [ $? -ne 0 ]; then
		printf -v file_number "%0${n_digits}i" 1
	else
		file_number=${file_number: -$n_digits:$n_digits}
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
	cp -r $tmp_storage_dir/$model_dir $extraction_dir/$model_dir'_'$file_number
	echo "$file_number -- ${log[*]} => ${result[*]}" >> $extraction_dir/index.log
}


# Initialize the file_number:
get_file_number

# Synchronous updates:
while inotifywait -qqe modify $watched_file; do
	if [ $((++skip_count)) -ge $skip ]; then
		skip_count=0

		log=($( tail -1 $watched_file ))

		cp -r $session_dir/$model_dir $tmp_storage_dir
		echo -ne "> Evaluating ${log[*]::2}...\r"
		result=($(../build/rover_training_1_exe eval $tmp_storage_dir/$model_dir 2> /dev/null))
		stats="${log[*]} => ${result[*]}"

		if [ "$display_only" == true ]; then
			echo $stats
		else
			echo $stats | tee -a $session_dir/stats.log
			if [[ ${result[*]} == *'[Success]'* ]]; then
				if (( $( echo "${result[$duration_pos]} <= $target_duration" | bc -l ) )); then
					extract_data
					echo "Dataset $file_number extracted (time: ${result[$duration_pos]})"
				fi
			fi
		fi
	fi
done
