rtmros_tutorials  [![Build Status](https://travis-ci.org/start-jsk/rtmros_tutorials.png)](https://travis-ci.org/start-jsk/rtmros_tutorials)
--------------

Tutorials for rtmros packages

### Install
```
mkdir -p ~/ros/ws_rtmros_tutorials/src
cd ~/ros/ws_rtmros_tutorials/src
wstool init
wstool set rtm-ros-robotics/rtmros_tutorials https://github.com/start-jsk/rtmros_tutorials.git --git -y
wstool update
cd ~/ros/ws_rtmros_tutorials/
source /opt/ros/hydro/setup.bash
catkin_make --only-pkg-with-deps hrpsys_gazebo_tutorials
source ~/ros/ws_rtmros_tutorials/devel/setup.bash # this line is necessary for bug, need to be fixed.
catkin_make --only-pkg-with-deps hrpsys_gazebo_tutorials --force-cmake # this line is necessary for bug, need to be fixed.
catkin_make --only-pkg-with-deps hrpsys_gazebo_tutorials --force-cmake # this line is necessary for bug, need to be fixed.
echo "source ~/ros/ws_rtmros_tutorials/devel/setup.bash" >> ~/.bashrc
echo "source \`rospack find hrpsys_gazebo_tutorials\`/setup.sh" >> ~/.bashrc
```

### Install Depend Package
```
sudo apt-get install python-rosdep
sudo rosdep init
roscd hrpsys_ros_bridge_tutorials/..
find -L . -name package.xml -exec dirname {} \; | xargs -n 1 -i find {} -name manifest.xml | xargs -n 1 -i mv {} {}.deprecated # rename manifest.xml for rosdep install
rosdep install -r -n --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
find -L . -name manifest.xml.deprecated | xargs -n 1 -i dirname {} | xargs -n 1 -i mv `pwd`/{}/manifest.xml.deprecated `pwd`/{}/manifest.xml

```

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

hrpsys_ros_bridge_tutorialsls
--------------
This is examples and tutorials for [rtmros_common/hrpsys_ros_bridge](https://github.com/start-jsk/rtmros_common). 

Euslisp tutorial wiki is written in [ROS wiki](http://wiki.ros.org/rtmros_common/Tutorials/WorkingWithEusLisp).

