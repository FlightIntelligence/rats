#!/bin/zsh

roslaunch BebopExperiment.launch &
rosrun dynamic_reconfigure dynparam load /bebop/bebop_driver bebop.yaml -t 10 ;
fg