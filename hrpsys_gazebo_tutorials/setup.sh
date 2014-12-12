#!/bin/sh

source `rospack find hrpsys_gazebo_general`/setup.sh

##
tpkgdir=`rospack find hrpsys_gazebo_tutorials`

if [ -e ${tpkgdir} ]; then
    export GAZEBO_RESOURCE_PATH=${tpkgdir}/worlds:$GAZEBO_RESOURCE_PATH
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${tpkgdir}/robot_models:${tpkgdir}/environment_models:${tpkgdir}/..
fi
