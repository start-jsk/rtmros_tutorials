#!/bin/bash

source ~/.bashrc

pgrep -f roslaunch.*hrp$(expr $HRP2NO + 2000)_gazebo.launch | xargs -n 1 kill
#pgrep -f roslaunch.*hrp$(expr $HRP2NO + 2000)_ros_bridge.launch | xargs -n 1 kill
pgrep -f roslaunch.*hrp$(expr $HRP2NO + 2000)_auditor_gazebo.launch | xargs -n 1 kill
