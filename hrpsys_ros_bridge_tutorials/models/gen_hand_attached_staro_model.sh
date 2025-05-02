#!/bin/bash

function error {
    exit 1
}
trap error ERR

ROBOT_MODEL=$1
INPUT_FILE=$2
EUSCOLLADA_PATH=$3
BODY_FILE=${INPUT_FILE//.urdf/_body.urdf}

# enable to execute with correct python version
# cf. https://github.com/tork-a/openrtm_aist_python-release/pull/5
if command -v python3 &>/dev/null; then
    PYTHON_EXECUTABLE=$(command -v python3)
elif command -v python &>/dev/null; then
    PYTHON_EXECUTABLE=$(command -v python)
else
    echo "Cannot find python executable"
    exit 1
fi

tmp1=`mktemp`
tmp2=`mktemp`
${PYTHON_EXECUTABLE} ${EUSCOLLADA_PATH}/scripts/remove_sensor_from_urdf.py LARM_LINK7 $INPUT_FILE $tmp1
${PYTHON_EXECUTABLE} ${EUSCOLLADA_PATH}/scripts/remove_sensor_from_urdf.py RARM_LINK7 $tmp1 $BODY_FILE
