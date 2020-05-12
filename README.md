rtmros_tutorials  [![Build Status](https://travis-ci.org/start-jsk/rtmros_tutorials.png)](https://travis-ci.org/start-jsk/rtmros_tutorials)
--------------

Tutorials for rtmros packages

### Install

Please refer [rtmros_common] for installing these packages.

If you use closed models, you have to compile `hrpsys_ros_bridge_tutorials` after you download `jsk_models` or `hrp2_models`. If you compiled `hrpsys_ros_bridge_tutorials` before you download `jsk_models` or `hrp2_models`, you have to compile `hrpsys_ros_bridge_tutorials` again with `--force-cmake` option after you download `jsk_models` or `hrp2_models`.

**NOTE** If you use `melodic` distribution, you have to compile `hrpsys_ros_bridge_tutorials` after you install `ros-melodic-collada-urdf-jsk-patch`, which is automatically installed with `rosdep install` for `hrpsys_ros_bridge_tutorials`.

### Try Sample
#### SampleRobot
Open Terminal and run gazebo

```
roslaunch hrpsys_gazebo_tutorials gazebo_samplerobot_no_controllers.launch #kinetic or melodic
roslaunch hrpsys_gazebo_tutorials gazebo_samplerobot_no_controllers_indigo.launch #indigo
```
Launch another terminal and start hrpsys-base
```
rtmlaunch hrpsys_gazebo_tutorials samplerobot_hrpsys_bringup.launch
```
Launch another terminal and send command to robot by roseus
```
roscd hrpsys_ros_bridge_tutorials/euslisp/
roseus samplerobot-interface.l
(samplerobot-init)
(send *ri* :angle-vector (send *sr* :reset-pose) 1000)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)
(send *ri* :go-pos 0 0 0)
```

#### HRP2
`rtmros_hrp2`(private) is needed.
Open Terminal and run gazebo

```
roslaunch hrpsys_gazebo_tutorials gazebo_hrp2jsknt_no_controllers.launch #kinetic or melodic
roslaunch hrpsys_gazebo_tutorials gazebo_hrp2jsknt_no_controllers_indigo.launch #indigo
```
Launch another terminal and start hrpsys-base
```
rtmlaunch hrpsys_gazebo_tutorials hrp2jsknt_hrpsys_bringup.launch
```
Launch another terminal and send command to robot by roseus
```
roscd hrpsys_ros_bridge_tutorials/euslisp/
roseus hrp2jsknt-interface.l
(hrp2jsknt-init)
(send *ri* :angle-vector (send *hrp2jsknt* :reset-pose) 1000)
(send *ri* :start-auto-balancer)
(send *ri* :start-st)
(send *ri* :go-pos 0 0 0)
```


### Package Description

### [hrpsys_ros_bridge_tutorials]
--------------
This is examples and tutorials for [rtmros_common/hrpsys_ros_bridge](https://github.com/start-jsk/rtmros_common).

Euslisp tutorial wiki is written in [ROS wiki](http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp).

- You should prepare robot model file (no additional operation is required for `SampleRobot`, downloading `hrp2_models`(private) is required for `HRP2`). Supported types of model file are collada(openrave) and VRML(openhrp3). URDF and OpenRAVE xml can be used by converting to collada.
    - *&lt;robot_name&gt;*.yaml for configurating URDF setting, gazebo setting and euslisp setting
    - (automatically generated) *&lt;robot_name&gt;*.urdf under models directory
    - (automatically generated) hrpsys settings (you should have a collada or VRML robot model file)

### [hrpsys_gazebo_tutorials]

This package is a collection of examples for using hrpsys_gazebo system and utility scripts.
Please see README in that package for further information.

[rtmros_common]:https://github.com/start-jsk/rtmros_common
[hrpsys_gazebo_tutorials]:https://github.com/start-jsk/rtmros_tutorials/tree/master/hrpsys_gazebo_tutorials
[hrpsys_ros_bridge_tutorials]:https://github.com/start-jsk/rtmros_tutorials/tree/master/hrpsys_ros_bridge_tutorials
