#!/bin/bash

set -e

# setup arguments
robot_name=$1
eusurdf_path=$2
input_urdf=$3
output_urdf=$4
yaml_file=$5

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

# generating model with sensors
function add_sensor_to_tmp_urdf()
{
    tmp_model=`mktemp`
    ${PYTHON_EXECUTABLE} ${eusurdf_path}/scripts/add_sensor_to_urdf.py $@ $tmp_model
    if [ $? == 0 ]; then
        echo $tmp_model
    else
        return $?
    fi
}

function add_eef_to_tmp_urdf()
{
    tmp_model=`mktemp`
    ${PYTHON_EXECUTABLE} ${eusurdf_path}/scripts/add_eef_to_urdf.py $@ $tmp_model
    if [ $? == 0 ]; then
        echo $tmp_model
    else
        return $?
    fi
}

## STARO
function generate_staro_model()
{
#  # force sensors
#     tmp_model0=`add_sensor_to_tmp_urdf 0 0 -0.069 3.14159 0 3.14159 LLEG_LINK5 lfsensor $1`
#     echo Added lfsensor $tmp_model0
#     tmp_model1=`add_sensor_to_tmp_urdf 0 0 -0.069 3.14159 0 3.14159 RLEG_LINK5 rfsensor $tmp_model0`
#     echo Added rfsensor $tmp_model1
#     tmp_model2=`add_sensor_to_tmp_urdf 0  0.05755 0 -1.5708 0 0 LARM_LINK7 lasensor $tmp_model1`
#     echo Added lhsensor $tmp_model2
#     tmp_model3=`add_sensor_to_tmp_urdf 0 -0.05755 0 0.78539958 -1.57080033 0.78539958 RARM_LINK7 rasensor $tmp_model2`
#     echo Added rhsensor $tmp_model3
# # end coords
#     tmp_model4=$(add_eef_to_tmp_urdf $tmp_model3 $3)
#     echo Added eef $tmp_model4
# # camera
#     echo Adding head
#     # tmp_model5=$(add_sensor_to_tmp_urdf 0.093 0.017 0.131 -1.787 0.038 -1.576 HEAD_LINK1 multisense $tmp_model4)
#     # tmp_model6=$(add_sensor_to_tmp_urdf -0.045 0.0 0.0 1.571 -1.571 0.0 CARMINE camera_link $tmp_model5)
#     # tmp_model7=$(add_sensor_to_tmp_urdf 0 -0.045 0 0 0 0 camera_link camera_rgb_frame $tmp_model6)
#     # tmp_model8=$(add_sensor_to_tmp_urdf 0 0 0 -1.571 0 -1.571 camera_rgb_frame camera_rgb_optical_frame $tmp_model7)
#     # tmp_model9=$(add_sensor_to_tmp_urdf 0 -0.02 0.0 0 0 0 camera_link camera_depth_frame $tmp_model8)
#     # tmp_model10=$(add_sensor_to_tmp_urdf 0 0 0 -1.571 0 -1.571 camera_depth_frame camera_depth_optical_frame $tmp_model9)
#     # # add calib jig
#     # tmp_model11=$(add_sensor_to_tmp_urdf 0 0.2 -0.2 0 0 1.57 LARM_LINK6 LARM_cb_jig $tmp_model10)
#     # tmp_model12=$(add_sensor_to_tmp_urdf 0 -0.2 -0.2 0 0 1.57 RARM_LINK6 RARM_cb_jig $tmp_model11)
#     cp $tmp_model4 $2
    ${PYTHON_EXECUTABLE} ${eusurdf_path}/scripts/add_sensor_to_collada.py $1 -O $2 -C $3
}

generate_staro_model $input_urdf $output_urdf $yaml_file
