# Dynamics environment for reinforcement learning

<p align="center">
	<img src="cover_picture.png?raw=true" width="600">
</p>

Dynamics environment for reinforcement learning with Open Dynamics Engine and TensorFLow.<br />
The environment mixes dynamic simulation compiled from C++ and scripts in Python and Bash.


### Dependencies:

To compile the simulations, you will need to install the following libraries:  
`$ sudo apt-get install libode-dev libopenscenegraph-dev libeigen3-dev libboost-dev libboost-python-dev libyaml-cpp-dev`

You will also need to build a C++ shared library of TensorFlow 1.13.1, which can be done without Bazel from [this git](https://github.com/FloopCZ/tensorflow_cc "github.com/FloopCZ/tensorflow_cc") using the tag `v1.13.1`:  
`$ git clone -b v1.13.1 --depth=1 https://github.com/FloopCZ/tensorflow_cc`  
The building instructions for TensorFLow can then be found in *tensorflow_cc/README.md*.

For the reinforcement learning scripts, you will need Tensorflow 1.13.1 for Python, as well as numpy and tqdm:  
`$ sudo pip install tensorflow==1.13.1 numpy tqdm`


### Build the simulations:

`$ mkdir build && cd build`  
`$ cmake ..`  
`$ make`  


### Start a training:

Set the identifier directory name for the training data in both *rover_training_1.py* and *eval_and_extract_policies.sh*. For example:  
- `run_id = 'run_1'` in *rover_training_1.py*.
- `run_id=run_1` in *eval_and_extract_policies.sh*.

If the directory doesn't exist yet, create it so that tee can start writing in it:  
`$ mkdir -p ../training_data/run_1`

Then, start the training with:  
`$ ./rover_training_1.py | tee -i ../training_data/run_1/rover_training_1_log.txt`

In order to monitor the progress, run in another terminal:  
`$ ./eval_and_extract_policies.sh`  
The first argument for this script can be either the number of model updates to skip between two evaluations or `-d, --display-only` in order to avoid recording the stats and backing up models.
