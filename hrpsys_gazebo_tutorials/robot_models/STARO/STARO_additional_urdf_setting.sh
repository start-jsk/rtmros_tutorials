#!/bin/bash

function error {
    exit 1
}
trap error ERR

OUTPUT_FILE=$1
## change foot parameters
sed -i -e '/<gazebo reference="LLEG_LINK5">/{N;N;N;N;s@  <gazebo reference="LLEG_LINK5">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="LLEG_LINK5">\n    <kp>1000000.0</kp>\n    <kd>800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
sed -i -e '/<gazebo reference="RLEG_LINK5">/{N;N;N;N;s@  <gazebo reference="RLEG_LINK5">\n    <mu1>0.9</mu1>\n    <mu2>0.9</mu2>\n  </gazebo>@  <gazebo reference="RLEG_LINK5">\n    <kp>1000000.0</kp>\n    <kd>800.0</kd>\n    <mu1>1.5</mu1>\n    <mu2>1.5</mu2>\n    <fdir1>1 0 0</fdir1>\n    <maxVel>10.0</maxVel>\n    <minDepth>0.00</minDepth>\n  </gazebo>@;}' ${OUTPUT_FILE}
## change foot geometry to BOX
sed -i -e '/<collision>/{N;N;N;s@<collision>\n      <origin xyz="0 0 -0.04" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://hrpsys_gazebo_tutorials/robot_models/STARO/meshes/RLEG_LINK5_mesh.dae" scale="1 1 1" />@<collision>\n      <origin xyz="0.015 -0.010 -0.0709961" rpy="0 -0 0"/>\n      <geometry>\n        <box size="0.23 0.13 0.05" />@;}' ${OUTPUT_FILE}
sed -i -e '/<collision>/{N;N;N;s@<collision>\n      <origin xyz="0 0 -0.04" rpy="0 -0 0"/>\n      <geometry>\n        <mesh filename="package://hrpsys_gazebo_tutorials/robot_models/STARO/meshes/LLEG_LINK5_mesh.dae" scale="1 1 1" />@<collision>\n      <origin xyz="0.015 0.010 -0.0709961" rpy="0 -0 0"/>\n      <geometry>\n        <box size="0.23 0.13 0.05" />@;}' ${OUTPUT_FILE}
# continuous joint not working in GAZEBO
sed -i -e 's@continuous@revolute@g' ${OUTPUT_FILE}
sed -i -e 's@<joint name="HEAD_JOINT0" type="fixed">@<joint name="HEAD_JOINT0" type="revolute">@g' ${OUTPUT_FILE}
