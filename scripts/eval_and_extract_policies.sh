#!/bin/bash

if [[ -z $TRAINING_DATA_DIR && -z $BUILD_DIR ]]; then
	>&2 echo The environment variables has not been set. You might need to source setup.sh.
	exit 1
fi


############################
# Default parameter values #
############################

# Maximum duration of a successful run for it to be extracted:
target_duration=40

# Maximum amount of data to extract:
max_extractions=10

# Number of digits for the identifiers of extracted models:
n_digits=2

# Position of the trial duration in the string returned from the evaluation:
duration_pos=2

# Number of updates between two evaluations:
eval_freq=1


############################
# Parsing of the arguments #
############################

check_positive_integer()
{
	if ! [[ $1 =~ ^[0-9]+$ ]]; then
		>&2 echo Expected a positive integer but received: $1
		exit 1
	fi
}

check_positive_float()
{
	if ! [[ $1 =~ ^[0-9]+\.?[0-9]*$ ]]; then
		>&2 echo Expected a positive float but received: $1
		exit 1
	fi
}

if [[ $# -lt 1 ]]; then
	>&2 echo Please specify the executable file that performs the evaluation.
	exit 1
fi
eval_exe_file=$BUILD_DIR$1
shift

if [[ $# -lt 1 ]]; then
	>&2 echo Please specify the identification name of the training.
	exit 1
fi
session_id=$1
shift

while [[ $# -gt 0 ]]; do

	case $1 in

	-n|--no-extraction)
		no_extraction=true
		echo -- No extraction
		;;

	-t|--target-duration)
		shift
		check_positive_float $1
		target_duration=$1
		echo -- Target duration: $target_duration
		;;

	-m|--max-extractions)
		shift
		check_positive_integer $1
		max_extractions=$1
		echo -- Maximum number of extractions: $max_extractions
		;;

	-f|--eval-freq)
		shift
		check_positive_integer $1
		eval_freq=$1
		echo -- Evaluation frequency: $eval_freq
		;;

	*)
		>&2 echo Unknown argument: $1
		exit 1

	esac

	shift

done


###################
# Initializations #
###################

session_dir=$TRAINING_DATA_DIR$session_id
watched_file=$session_dir/training.log
model_dir=actor
extraction_dir=$session_dir/picked
tmp_storage_dir=/tmp/${session_id}_tmp_model

export TF_CPP_MIN_LOG_LEVEL=1

skip_count=$eval_freq

# Create the expression for the number of digits looked for:
for (( i = 0 ; i < n_digits ; i++ )); do
	digit_expr+=[0-9]
done

get_file_number()
{
	file_number=$( ls -d $extraction_dir/$model_dir'_'$digit_expr 2> /dev/null )

	if [[ $? -ne 0 ]]; then
		printf -v file_number "%0${n_digits}i" 1
	else
		file_number=${file_number: -$n_digits:$n_digits}
		file_number=$(( 10#$file_number + 1 ))
		printf -v file_number "%0${n_digits}i" $file_number
	fi
}

# Initialize the file_number:
get_file_number


###############################
# Evaluations and extractions #
###############################

# Create the temporary storage directory:
mkdir -p $tmp_storage_dir

extract_data()
{
	if [[ $((++c)) -le $max_extractions ]]; then
		get_file_number
	fi

	mkdir -p $extraction_dir
	cp -r $tmp_storage_dir/$model_dir $extraction_dir/$model_dir'_'$file_number
	echo "$file_number -- ${log[*]} => ${result[*]}" >> $extraction_dir/index.log
}

# Wait for file updates:
while inotifywait -qqe modify $watched_file; do

	if [[ $((++skip_count)) -ge $eval_freq ]]; then
		skip_count=0

		log=( $( tail -1 $watched_file ) )

		# Copy the actor model in the temporary directory and run its evaluation:
		cp -r $session_dir/$model_dir $tmp_storage_dir
		echo -ne "> Evaluating ${log[*]::2}...\r"
		result=( $( $eval_exe_file eval $tmp_storage_dir/$model_dir 2> /dev/null ) )
		stats="${log[*]} => ${result[*]}"

		# Log the result of the evaluation:
		echo $stats | tee -a $session_dir/stats.log

		if [[ $no_extraction != true ]]; then
			if [[ ${result[*]} == *'[Success]'* ]]; then
				if (( $( echo "${result[$duration_pos]} <= $target_duration" | bc -l ) )); then
					extract_data
					echo "Dataset $file_number extracted (time: ${result[$duration_pos]})"
				fi
			fi
		fi
	fi
done
