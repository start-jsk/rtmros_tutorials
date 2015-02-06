#!/bin/bash

set -e

# setup arguments
eusurdf_path=$1
input_urdf=$2
output_urdf=$3
yaml_file=$4

# generating model with sensors
function add_sensor_to_tmp_urdf()
{
    tmp_model=`mktemp`
    ${eusurdf_path}/scripts/add_sensor_to_urdf.py $@ $tmp_model
    if [ $? == 0 ]; then
        echo $tmp_model
    else
        return $?
    fi
}

function add_eef_to_tmp_urdf()
{
    tmp_model=`mktemp`
    ${eusurdf_path}/scripts/add_eef_to_urdf.py $@ $tmp_model
    if [ $? == 0 ]; then
        echo $tmp_model
    else
        return $?
    fi
}

# force sensors
# LLEG_LINK5 -> lfsensor
# RLEG_LINK5 -> rfsensor
# RARM_LINK6 -> rhsensor
# LARM_LINK6 -> lhsensor

# camera
# HEAD_LINK1 -> CARMINE
# CARMINE -> camera_link
# camera_link -> camera_rgb_frame
# camera_rgb_frame -> camera_rgb_optical_frame
# camera_link -> camera_depth_frame
# camera_depth_frame -> camera_depth_optical_frame

# end coords
# LARM_LINK6 -> larm_end_coords
# RARM_LINK6 -> rarm_end_coords
# LLEG_LINK5 -> lleg_end_coords
# RLEG_LINK5 -> rleg_end_coords
# HEAD_LINK1 -> head_end_coords

function generate_hrp2wjsknt_model()
{
 # force sensors
    #tmp_model0=`add_sensor_to_tmp_urdf 0 0 -0.105 0 0 0 LLEG_LINK5 lfsensor $1`
    #echo Added lfsensor $tmp_model0
    #tmp_model1=`add_sensor_to_tmp_urdf 0 0 -0.105 0 0 0 RLEG_LINK5 rfsensor $tmp_model0`
    #echo Added rfsensor $tmp_model1
    tmp_model2=`add_sensor_to_tmp_urdf 0 0 -0.077 0 0 0 LARM_LINK6 lhsensor $1`
    echo Added lhsensor $tmp_model2
    tmp_model3=`add_sensor_to_tmp_urdf 0 0 -0.077 0 0 0 RARM_LINK6 rhsensor $tmp_model2`
    echo Added rhsensor $tmp_model3
# end coords
    #tmp_model4=$(add_eef_to_tmp_urdf $tmp_model3 $3)
    #echo Added eef $tmp_model4
# camera
    echo Adding head
    tmp_model5=$(add_sensor_to_tmp_urdf 0.093 0.017 0.131 -1.787 0.038 -1.576 HEAD_LINK1 CARMINE $tmp_model3)
    tmp_model6=$(add_sensor_to_tmp_urdf -0.045 0.0 0.0 1.571 -1.571 0.0 CARMINE camera_link $tmp_model5)
    tmp_model7=$(add_sensor_to_tmp_urdf 0 -0.045 0 0 0 0 camera_link camera_rgb_frame $tmp_model6)
    tmp_model8=$(add_sensor_to_tmp_urdf 0 0 0 -1.571 0 -1.571 camera_rgb_frame camera_rgb_optical_frame $tmp_model7)
    tmp_model9=$(add_sensor_to_tmp_urdf 0 -0.02 0.0 0 0 0 camera_link camera_depth_frame $tmp_model8)
    tmp_model10=$(add_sensor_to_tmp_urdf 0 0 0 -1.571 0 -1.571 camera_depth_frame camera_depth_optical_frame $tmp_model9)
    # add calib jig
    tmp_model11=$(add_sensor_to_tmp_urdf 0 0.2 -0.2 0 0 1.57 LARM_LINK6 LARM_cb_jig $tmp_model10)
    tmp_model12=$(add_sensor_to_tmp_urdf 0 -0.2 -0.2 0 0 1.57 RARM_LINK6 RARM_cb_jig $tmp_model11)
    cp $tmp_model12 $2
}

generate_hrp2wjsknt_model $input_urdf $output_urdf $yaml_file
