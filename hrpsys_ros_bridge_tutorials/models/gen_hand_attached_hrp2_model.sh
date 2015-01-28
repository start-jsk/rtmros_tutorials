#!/bin/bash

function error {
    exit 1
}
trap error ERR

ROBOT_MODEL=$1
INPUT_FILE=$2
EUSCOLLADA_PATH=$3
BODY_FILE=${INPUT_FILE//.urdf/_body.urdf}

tmp1=`mktemp`
echo INPUT_FILE is $INPUT_FILE
${EUSCOLLADA_PATH}/scripts/remove_sensor_from_urdf.py LARM_LINK6 $INPUT_FILE $tmp1
${EUSCOLLADA_PATH}/scripts/remove_sensor_from_urdf.py RARM_LINK6 $tmp1 $BODY_FILE
