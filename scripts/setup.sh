# Bash script providing the auto completion for the scripts
# train.sh, eval_and_extract_policies.sh and eval_policy.sh.

# Environment variable specifying the directory in which to store the training data
# (include a final slash at the end):
export TRAINING_DATA_DIR=../training_data/

# Environment variable specifying the directory where to find the compiled simulations
# (include a final slash at the end):
export BUILD_DIR=../build/



_train()
{
	local choice

	case $COMP_CWORD in

	1)
		# List the Python scripts in the current directory and keep only their base name:
		choice=$( ( find . -maxdepth 1 -type f -name '*.py' | xargs basename -a ) 2>/dev/null )
		;;

	2)
		# List the directories in TRAINING_DATA_DIR and keep only their base name:
		choice=$( ( find $TRAINING_DATA_DIR -mindepth 1 -maxdepth 1 -type d | xargs basename -a ) 2>/dev/null )
		;;

	3)
		choice='resume'

	esac

	COMPREPLY=( $(compgen -W '$choice' -- ${COMP_WORDS[COMP_CWORD]}) )
}

complete -F _train './train.sh'


_eval_and_extract_policies()
{
	local choice

	case $COMP_CWORD in

	1)
		# List the executable files in BUILD_DIR and keep only their base name:
		choice=$( ( find $BUILD_DIR -maxdepth 1 -type f -executable | xargs basename -a ) 2>/dev/null )
		;;

	2)
		# List the directories in TRAINING_DATA_DIR and keep only their base name:
		choice=$( ( find $TRAINING_DATA_DIR -mindepth 1 -maxdepth 1 -type d | xargs basename -a ) 2>/dev/null )
		;;

	*)
		choice='--no-extraction --target-duration --max-extractions --eval-freq'

	esac

	COMPREPLY=( $(compgen -W '$choice' -- ${COMP_WORDS[COMP_CWORD]}) )
}

complete -F _eval_and_extract_policies './eval_and_extract_policies.sh'


_eval_policy()
{
	local choice

	case $COMP_CWORD in

	1)
		# List the executable files in BUILD_DIR and keep only their base name:
		choice=$( ( find $BUILD_DIR -maxdepth 1 -type f -executable | xargs basename -a ) 2>/dev/null )
		;;

	2)
		# List the directories in TRAINING_DATA_DIR and keep only their base name:
		choice=$( ( find $TRAINING_DATA_DIR -mindepth 1 -maxdepth 1 -type d | xargs basename -a ) 2>/dev/null )
		;;

	3)
		choice='--picked-policy'
		;;

	4)
		prev_arg=${COMP_WORDS[COMP_CWORD-1]}
		if [[ $prev_arg == -p || $prev_arg == --picked-policy ]]; then
			session_id=${COMP_WORDS[COMP_CWORD-2]}
			# List the directories in the picked directory of the desired session which names begin with 'actor_' and keep only the suffixes:
			choice=$( ( find $TRAINING_DATA_DIR$session_id/picked -maxdepth 1 -type d -name actor_* | xargs basename -a | sed -e 's:actor_::' ) 2>/dev/null )
		fi

	esac

	COMPREPLY=( $(compgen -W '$choice' -- ${COMP_WORDS[COMP_CWORD]}) )
}

complete -F _eval_policy './eval_policy.sh'
