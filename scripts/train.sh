#!/bin/bash

if [[ -z $TRAINING_DATA_DIR && -z $BUILD_DIR ]]; then
	>&2 echo The environment variables has not been set. You might need to source setup.sh.
	exit 1
fi


if [[ $# -lt 1 ]]; then
	>&2 echo Please specify the training script.
	exit 1
fi
training_script=$1
shift

if [[ $# -lt 1 ]]; then
	>&2 echo Please specify the identification name of the training.
	exit 1
fi
session_dir=$TRAINING_DATA_DIR$1
shift


mkdir -p $session_dir

python $training_script $session_dir $* | tee -i $session_dir/training.log
