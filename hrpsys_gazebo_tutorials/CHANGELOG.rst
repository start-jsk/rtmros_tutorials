^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_gazebo_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* [hrpsys_gazebo_tutorials] split out gazebo_multisense.launch
* [hrpsys_gazebo_tutorials] add ENABLE_PIN_MODE option to jaxon launch file
* [hrpsys_gazebo_tutorials] fix typo in gazebo_jaxon_no_controllers.launch
* [hrpsys_gazebo_tutorials] add launch that enables hokuyo of multisense
* apply end effector changes from JAXON_without_multisense to JAXON
* add arguments to initial robot pose for jaxon
* add instruction to launch JAXON teleop
* add end-effector joint and links to jaxon xacro. these are added automatically from yaml files in hrpsys_ros_bridge_tutorials, so this change is temporary, and we should use that model.
* add multisense urdf of real robot to jaxon_without_multisense xacro
* fix gazebo files for jaxon simulation
* Contributors: Masaki Murooka, Ryohei Ueda, Yuki Furuta

0.1.5 (2015-04-27)
------------------
* [hrpsys_gazebo_tutorials/CMakeLists.txt] setup.sh is removed in #251
* [hrpsys_gazebo_tutorials] add DRCTestbed environement, world and launch files for gazebo/samplerobot
* [hrpsys_gazebo_tutorials] use env-hook instead of source setup.sh manually
* [hrpsys_gazebo_tutorials] add initial pose option for gazebo/samplerobot launch
* (hrpsys_gazebo_tutorials/euslisp/hand-command-publisher.l): Modified l/r correspondance which was inconsistent. Current consistency is "left is 0 and right is 1".
* Contributors: Furushchev, Kei Okada, leus

0.1.4 (2015-04-07)
------------------
* Remove old manifest.xml files
* [rtmros_tutorials] remove old rosbuild settings
* [hrpsys_gazebo_tutorials] Fix path for catkin build
* fix head position of staro for gazebo
* launch/staro_hrpsys_bringup.launch: Load STARO_controller_config.yaml when start hrpsys for Seq partial controllers
* add jaxon
* update for using staro in drc world
* add variable for cheat
* add files for using drc_world with STARO
* update setup.sh
* fix drill color
* add hand command to staro-interface.l
* add controller configuration yaml for ROBOTIQ
* update launch for using multisense and hands
* add robotiq directory
* add multisense to staro.urdf.xacro for gazebo
* add drill VRML model
* fixed mistake: L_ -> R_
* not generate special world file for each robot.
* restore world_source.world
* make hrpsys_gazebo_tutorials in travis. add hrpsys_gazebo_general to catkin dependency in hrpsys_gazebo_tutorials
* fix initial pose and use cfm
* add stero setting for gazebo
* move hrpsys_gazebo_tutorials to subdirectory
* Contributors: Kei Okada, Ryohei Ueda, YoheiKakiuchi, iori, mmurooka
