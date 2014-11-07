#!/bin/bash

function error {
    exit 1
}
trap error ERR

ROBOT_MODEL=$1
INPUT_FILE=$2
ADDITIONAL_ROS_PACKAGE_PATH=$3
BODY_FILE=`echo ${INPUT_FILE} | sed "s/.urdf/_body.urdf/g"`
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ADDITIONAL_ROS_PACKAGE_PATH

# wait until input files are generated
echo "wait to generate ${INPUT_FILE} and ${LAUNCH_FILE}"
while [ ! -e ${INPUT_FILE} -o ! -e ${LAUNCH_FILE} -o ! -e `rospack find hrpsys_ros_bridge_tutroials`/models/HRP3HAND_L.urdf -o ! -e `rospack find hrpsys_ros_bridge_tutroials`/models/HRP3HAND_R.urdf]
do
    sleep 1
done

# make tmp file
cp ${INPUT_FILE} ${BODY_FILE}

# remove LARM_LINK6
L_START=`grep -n "<link name=\"LARM_LINK6\"" -m 1 ${BODY_FILE} -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" ${BODY_FILE} | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:) ##
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" ${BODY_FILE}

# remove RARM_LINK6
L_START=`grep -n "<link name=\"RARM_LINK6\"" ${BODY_FILE} -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" ${BODY_FILE} | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:) ##
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" ${BODY_FILE}
