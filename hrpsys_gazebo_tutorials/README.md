## How to move SampleRobot in Gazebo with hrpsys-base
1. confirm if there is a SampleRobot.urdf in ```hrpsys_gazebo_tutorials/robot_models/SampleRobot```
1. ```source `rospack find hrpsys_gazebo_tutorials`/setup.sh; roslaunch hrpsys_gazebo_tutorials gazebo_samplerobot_no_controllers.launch```
1. ```rtmlaunch hrpsys_gazebo_tutorials samplerobot_hrpsys_bringup.launch```
1. ```roscd hrpsys_ros_bridge_tutorials/euslisp; roseus samplerobot-interface.l "(samplerobot-init)" "(objects (list *sr*))" "(send *ri* :angle-vector (send *sr* :angle-vector) 5000)"```

Do not forget to type ```pkill gzserver``` in addition to Ctrl-c in order to kill gazebo.

## How to test DRC Teleop Program in Gazebo with hrpsys-base
1. ```roslaunch hrpsys_gazebo_tutorials gazebo_jaxon_without_multisense_no_controllers.launch```
1. ```rtmlaunch hrpsys_gazebo_tutorials jaxon_hrpsys_bringup.launch```
1. ```roslaunch jsk_footstep_controller hrp2jsk_footcoords.launch```
1. ```roslaunch drc_task_common operator_station_main_jaxon.launch```
1. ```roslaunch drc_task_common field_computer_main.launch USE_LOCALHOST:=true```

Do not forget to type ```pkill gzserver``` in addition to Ctrl-c in order to kill gazebo.
Force sensor value is not valid.
