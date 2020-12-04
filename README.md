# Reinforcement learning framework harnessing efficient dynamic simulations in C++

<p align="center">
	<img src="cover_picture.png?raw=true" width="600">
</p>

Framework for reinforcement learning based on Open Dynamics Engine and TensorFLow.<br />
The framework mixes dynamic simulations in C++ and scripts in Python and Bash.


## Dependencies and compilation:

This repository depends on several git submodules. To download them, clone this repository with the option `git clone --recursive` or run in the root directory:  
`$ git submodule update --init`

For the reinforcement learning scripts, you will need the following packages for Python:  
`$ pip install tensorflow==2.3.1 tensorflow_probability tqdm PyYAML matplotlib`

To compile the simulations, you will need to install the following libraries for C++:  
`$ sudo apt-get install libode-dev libopenscenegraph-dev libeigen3-dev libboost-dev libboost-python-dev libyaml-cpp-dev`

### Build the TensorFlow shared library for C:

To build TensorFlow, you will need [Bazel](https://www.bazel.build). You can download a binary of Bazelisk [here](https://github.com/bazelbuild/bazelisk/releases), which is a wrapper for Bazel that will automatically pick a good version of Bazel given your current working directory. For convenience, you can put it in your `PATH` while renaming it `bazel`:  
`$ wget https://github.com/bazelbuild/bazelisk/releases/latest/download/bazelisk-linux-amd64`  
`$ chmod +x bazelisk-linux-amd64`  
`$ mkdir ~/bin`  
`$ PATH=$PATH:~/bin`  
`$ cp bazelisk-linux-amd64 ~/bin/bazel`

Clone TensorFlow sources:  
`$ git clone -b v2.3.1 --depth=1 https://github.com/tensorflow/tensorflow.git`

To avoid any conflict with the shared library *libtensorflow_framework.so* used by the TensorFlow package for Python, build the C library with the `monolithic` option:  
`$ cd tensorflow`  
`$ ./configure`  
`$ bazel build --config=monolithic //tensorflow:libtensorflow.so`

Install the headers via a symbolic link:  
`$ sudo ln -s $(pwd)/tensorflow /usr/local/include`

Install the shared library (`shopt -s extglob` enabled):  
`$ sudo cp -P bazel-bin/tensorflow/libtensorflow.so!(*params) /usr/local/lib`  
`$ sudo ldconfig`

### Build the simulations:

`$ mkdir build && cd build`  
`$ cmake ..`  
`$ make`



## Start a training:

Set the identifier directory name for the training data in both [rover_training_1.py](scripts/rover_training_1.py) and [eval_and_extract_policies.sh](scripts/eval_and_extract_policies.sh). For example:  
- `session_id = 'run_1'` in [rover_training_1.py](scripts/rover_training_1.py).
- `session_id=run_1` in [eval_and_extract_policies.sh](scripts/eval_and_extract_policies.sh).

If the directory doesn't exist yet, create it so that *tee* can start writing in it:  
`$ mkdir -p ../training_data/run_1`

Then, start the training with:  
`$ ./rover_training_1.py | tee -i ../training_data/run_1/training.log`

In order to monitor the progress, run in another terminal:  
`$ ./eval_and_extract_policies.sh`  
The first argument for this script can be either the number of model updates to skip between two evaluations or `-d, --display-only` in order to avoid recording the stats and backing up models.

To evaluate the current policy, simply run:  
`$ ./rover_training_1.py eval`
