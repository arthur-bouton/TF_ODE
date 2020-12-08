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

To monitor files during a training, you will also need the program `inotifywait`, which can be found in the following package:  
`$ sudo apt-get install inotify-tools`

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
`$ sudo ln -s $(realpath tensorflow) /usr/local/include`

Install the shared library (`shopt -s extglob` enabled):  
`$ sudo cp -P bazel-bin/tensorflow/libtensorflow.so!(*params) /usr/local/lib`  
`$ sudo ldconfig`

### Build the simulations:

`$ mkdir build && cd build`  
`$ cmake ..`  
`$ make`



## Start a training:

To set up the environment, source the setup script:  
`$ source scripts/setup.sh`  
To avoid doing it manually each time you open a new terminal, add it to your bashrc:  
`$ echo -e "\nsource $(realpath -s scripts/setup.sh)" >> ~/.bashrc`  
The setup.sh script gives you access to the commands `train`, `monitor-policies` and `eval-policy` from anywhere, together with the autocomplete. It also sets the following environment variables:
- `TRAINING_SCRIPTS_DIR`: Directory containing the training scripts.
- `TRAINING_DATA_DIR`: Directory in which to store the training data.
- `BUILD_DIR`: Directory where to find the compiled simulations.

To start a training with the identification name *run_1* for example, execute:  
`$ train rover_training_1.py run_1`

The training can be stopped with Ctrl-C and resumed with:  
`$ train rover_training_1.py run_1 resume`

In order to monitor the progress and backup well-performing policies, run in another terminal:  
`$ monitor-policies rover_training_1_exe run_1`  
Check the script [monitor-policies](scripts/bin/monitor-policies) for all the available options.

To evaluate the current policy, use:  
`$ eval-policy rover_training_1_exe run_1`  
To evaluate the picked policies by their number:  
`$ eval-policy rover_training_1_exe run_1 -p 01`
