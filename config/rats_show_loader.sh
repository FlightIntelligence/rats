#!/usr/bin/zsh

source /opt/ros/indigo/setup.zsh
source /home/controller/rosjava/devel/setup.zsh
source /home/controller/rats/rats_ws/devel/setup.zsh
source /home/controller/rats/bebop_ws/devel/setup.zsh
export LD_LIBRARY_PATH=/home/controller/rats/bebop_ws/devel/lib:/home/controller/rats/rats_ws/devel/lib:/home/controller/rosjava/devel/lib:/opt/ros/indigo/lib:/usr/lib/jni/

#test -e "${HOME}/.iterm2_shell_integration.zsh" && source "${HOME}/.iterm2_shell_integration.zsh"
export LD_LIBRARY_PATH=/opt/ibm/ILOG/CPLEX_Studio1263/cplex/bin/x86-64_linux/:/home/controller/rats/bebop_ws/devel/lib:/home/controller/rats/rats_ws/devel/lib:/home/controller/rosjava/devel/lib:/opt/ros/indigo/lib:/usr/lib/jni/

cd /home/controller/rats/config
python3 child_server.py 0.0.0.0 8080

