#!/bin/bash

FNAME1=/tmp/rosbridge_${USER}_`date +%Y-%m-%d_%H%M%S`.log
FNAME2=/tmp/auditor_${USER}_`date +%Y-%m-%d_%H%M%S`.log
roslaunch hrpsys_gazebo_tutorials hrp`expr $HRP2NO + 2000`_gazebo.launch 2>&1 | tee ${FNAME1} & roslaunch hrpsys_gazebo_tutorials hrp`expr $HRP2NO + 2000`_auditor_gazebo.launch 2>&1 | tee ${FNAME2}
