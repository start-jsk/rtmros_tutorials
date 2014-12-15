#!/bin/bash

project_dir=$(rospack find hrpsys_gazebo_tutorials)

input_file=$(rospack find drcsim_model_resources)/worlds/drc_practice_task_1.world
output_file=${project_dir}/worlds/$(basename ${input_file} .world)_staro.world

sed -e 's@<model_name>atlas@<model_name>STARO@g'  ${input_file} | \
sed -e 's@<cfm>0.0</cfm>@<cfm>10E-5</cfm>@g' | \
sed -e 's@<pin_link>utorso@<pin_link>BODY@g' > ${output_file}
