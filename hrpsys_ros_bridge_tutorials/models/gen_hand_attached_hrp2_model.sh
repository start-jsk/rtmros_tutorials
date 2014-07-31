#!/bin/bash

function error {
    exit 1
}
trap error ERR

ROBOT_MODEL=$1
INPUT_FILE=$2
OUTPUT_FILE=$3
LAUNCH_FILE=$4
ADDITIONAL_ROS_PACKAGE_PATH=$5
TMP_FILE=`echo ${INPUT_FILE} | sed "s/.urdf/_tmp.urdf/g"`
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ADDITIONAL_ROS_PACKAGE_PATH


# make tmp file
cp ${INPUT_FILE} ${TMP_FILE}

# remove LARM_LINK6
L_START=`grep -n "<link name=\"LARM_LINK6\"" -m 1 ${TMP_FILE} -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" ${TMP_FILE} | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:) ##
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" ${TMP_FILE}

# remove RARM_LINK6
L_START=`grep -n "<link name=\"RARM_LINK6\"" ${TMP_FILE} -m 1 | cut -f1 -d:`
L_END=$(sed -n "${L_START},\$p" ${TMP_FILE} | grep -n "<\/gazebo>" -m 1 | cut -f1 -d:) ##
L_END=`expr ${L_START} + ${L_END} - 1`
sed -i -e "${L_START},${L_END}d" ${TMP_FILE}

# generate URDF with xacro
rosrun xacro xacro.py `echo ${INPUT_FILE} | sed "s/.urdf/.urdf.xacro/g"` > ${OUTPUT_FILE}

# remove tmp file
rm -rf ${TMP_FILE}

# overwrite launch file to use new model
OUTPUT_FILE_BASENAME=`basename ${OUTPUT_FILE}`
sed -i -e "s@arg\ name=\"COLLADA_FILE\"\ \(.*\)${ROBOT_MODEL}.dae\"@arg\ name=\"COLLADA_FILE\"\ \\1${OUTPUT_FILE_BASENAME}\"@g" ${LAUNCH_FILE}
