FROM ubuntu:18.04

WORKDIR /root/TF_ODE

RUN apt-get update -qq
RUN apt-get install -y python3-pip cmake libode-dev libopenscenegraph-dev libeigen3-dev libboost-dev libboost-python-dev libyaml-cpp-dev inotify-tools bc

COPY requirements.txt .

RUN python3 -m pip install --upgrade pip
RUN pip3 install --no-cache-dir -r requirements.txt

COPY . /root/TF_ODE

RUN mv tensorflow_lib/tensorflow /usr/local/include/tensorflow
RUN mv tensorflow_lib/libtensorflow.so* /usr/local/lib/ && ldconfig

RUN mkdir build && cd build && cmake .. && make rover_training_1_module rover_training_1_exe && cd ..

RUN echo source $(realpath -s scripts/setup.sh) >> /root/.bashrc

RUN apt-get install -y tmux vim

CMD tmux new-session \; split-window -h \; resize-pane -t 0 -x 60 \; select-pane -t 0 \; attach ; bash
