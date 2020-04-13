rtmros_tutorials  [![Build Status](https://travis-ci.org/start-jsk/rtmros_tutorials.png)](https://travis-ci.org/start-jsk/rtmros_tutorials)
--------------

Tutorials for rtmros packages

### Install

Please refer [rtmros_common] for installing these packages.

If you use closed models, you have to compile `hrpsys_ros_bridge_tutorials` after you download `jsk_models` or `hrp2_models`. If you compiled `hrpsys_ros_bridge_tutorials` before you download `jsk_models` or `hrp2_models`, you have to compile `hrpsys_ros_bridge_tutorials` again with `--force-cmake` option after you download `jsk_models` or `hrp2_models`.

**NOTE** If you use `melodic` distribution, you have to compile `hrpsys_ros_bridge_tutorials` after you install `ros-melodic-collada-urdf-jsk-patch`, which is automatically installed with `rosdep install` for `hrpsys_ros_bridge_tutorials`.

### Try Sample
Open Terminal and run gazebo

```
roslaunch hrpsys_gazebo_tutorials gazebo_hrp2jsknt_no_controllers.launch
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
(send *ri* :go-pos 0 0 0)
```


### Package Description

### [hrpsys_ros_bridge_tutorials]
--------------
This is examples and tutorials for [rtmros_common/hrpsys_ros_bridge](https://github.com/start-jsk/rtmros_common).

Euslisp tutorial wiki is written in [ROS wiki](http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp).

### [hrpsys_gazebo_tutorials]

This package is a collection of examples for using hrpsys_gazebo system and utility scripts.

- You should prepare robot model file. Supported types of model file are collada(openrave) and VRML(openhrp3). URDF and OpenRAVE xml can be used by converting to collada.
    - *&lt;robot_name&gt;*.yaml for configurating gazebo setting and hrpsys setting
    - (automatically generated) *&lt;robot_name&gt;*.urdf under robot_models/*&lt;robot_name&gt;* directory
    - (automatically generated) hrpsys settings (you should have a collada or VRML robot model file)
    - *&lt;robot_name&gt;*_optional_urdf_setting.sh under robot_models/*&lt;robot_name&gt;* directory, this is for adding description used by gazebo (such as sensor settings, collision and friction setting)

#### (automatically generated files)

You can use robot_models/install_robot_common.sh for installing urdf model file. This scripts converts collada file in [hrpsys_ros_bridge_tutorials]/models directory to urdf file. 

    ./install_robot_common.sh ROBOT_NAME (model directory) (output directory) (collada_to_urdf_binary) (additional_ros_package_path)
