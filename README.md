# TF_ODE

Reinforcement learning with TensorFlow and Open Dynamics Engine


**Build the simulation:**

`$ mkdir build && cd build`
`$ cmake ..`
`$ make`


**Start a training:**

Set the identifier directory name for the training data in both *rover_training_1.py* and *eval_and_extract_policies.sh*, for example:
`run_id=run_1`

If the directory doesn't exist yet, make it so that tee can start writing in it:
`$ mkdir -p ../training_data/run_1`

Then, start the training with:
`$ ./rover_training_1.py | tee -i ../training_data/run_1/rover_training_1_log.txt`

In order to monitor progress, run in another terminal:
`$ ./eval_and_extract_policies.sh`
The first argument can be either the number of model updates to skip between two evaluations or `-d, --display-only` in order to avoid recording the stats and backing up models.
