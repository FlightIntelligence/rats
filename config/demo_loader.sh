#!/usr/bin/zsh

source /opt/ros/indigo/setup.zsh
source $HOME/rats/rats_ws/devel/setup.zsh
source $HOME/rats/bebop_ws/devel/setup.zsh
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/jni/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:~/JSCIPOpt/build
export ROS_IP=127.0.0.1


cd /home/controller/rats/config
python3 fidemo_child_server.py 0.0.0.0 8080

