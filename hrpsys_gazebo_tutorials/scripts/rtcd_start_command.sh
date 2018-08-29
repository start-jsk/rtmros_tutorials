#!/bin/bash

FNAME=/tmp/modelloader_${USER}_`date +%Y-%m-%d_%H%M%S`.log
rtmlaunch hrpsys_gazebo_tutorials modelloader.launch 2>&1 | tee ${FNAME} &

sleep 5

FNAME=/tmp/rtcd_${USER}_`date +%Y-%m-%d_%H%M%S`.log
TMP_RTC_LIST_SETTING=$(rosrun hrpsys_gazebo_tutorials hrp`expr $HRP2NO + 2000`_gazebo_setup.py --getRTCList | grep "example" | grep "config_file")

hrpsys_gazebo_tutorials_PATH=$(rospack find hrpsys_gazebo_tutorials)
rosrun hrpsys_gazebo_tutorials hrp`expr $HRP2NO + 2000`_gazebo_setup.py --init | tee /tmp/jsk_hrp2_setup_${USER}.log & bash -c "roslaunch hrpsys_gazebo_tutorials rtcd.launch ARGS:=\"-f ${hrpsys_gazebo_tutorials_PATH}/config/rtcdRobotModeHRP2Common.conf ${TMP_RTC_LIST_SETTING}\" 2>&1 | tee ${FNAME};"
