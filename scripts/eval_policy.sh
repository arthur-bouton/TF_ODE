#!/bin/bash

if [[ -z $TRAINING_DATA_DIR && -z $BUILD_DIR ]]; then
	>&2 echo The environment variables has not been set. You might need to source setup.sh.
	exit 1
fi


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
session_dir=$TRAINING_DATA_DIR$1
shift

if [[ $1 == -p || $1 == --picked-policy ]]; then
	shift
	if [[ $# -lt 1 ]]; then
		>&2 echo Please specify the number of the picked policy to evaluate.
		exit 1
	fi
	path_to_model_dir=$session_dir/picked/actor_$1
	shift
else
	path_to_model_dir=$session_dir/actor
fi


$eval_exe_file display $path_to_model_dir $*
