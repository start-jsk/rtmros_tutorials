^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package hrpsys_ros_bridge_tutorials
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* apply min-max table to STARO
* apply min-max table to JAXON
* enable gen_minmax_table_for_closed_robots for rbrain robots
* [hrpsys_ros_bridge_tutorials/CMakeLists.txt] revert wrong commit by `#219 <https://github.com/start-jsk/rtmros_tutorials/issues/219>`_
* [hrpsys_ros_bridge_tutorials/CMakeLists.txt] forget to convert CVSDIR -> hrp2_models_MODEL_DIR
* change multisense_s7 to sl
* [rtmros_tutorials] remove old rosbuild settings
* comment out method to call thk hand script because they are obsolated.
* [hrpsys_ros_bridge_tutorials] use hrp2_models_MODEL_DIR instad of ENV{CVSDIR}/OpenHRP/etc
* [hrpsys_ros_bridge_tutorials/package.xml] fix description
* [hrpsys_ros_bridge_tutorials/package.xml] add hrp2_models to depend
* add method definition for jaxon thk hand
* Merge pull request `#218 <https://github.com/start-jsk/rtmros_tutorials/issues/218>`_ from snozawa/update_mmtable_functions
  Merged.
* Remove unused codes and add check documentation
* Update min-max table function names and add documentation strings
* Do not check collision between toe link and shank link for hrp2jsknt and hrp2jsknts because these are not collide at ankle-p = 0
* Update default ee setting fot hrp2jsknt and hrp2jsknts. Use toe joint setting
* Fix order of walking sample
* update generation method for JAXON urdf
* Merge pull request `#211 <https://github.com/start-jsk/rtmros_tutorials/issues/211>`_ from mmurooka/add-jaxon-collision-pair
  Merged.
* add colision detector conf to jaxon opetion
* Add newline for mm table conf file line
* Fix initial joints for HRP2W simulation
* Fix HRP2W end coords according to real robot setting
* Add grasp controller setting for HRP2JSK
* Add conf file setting used in real robot conf files
* Delete definition of contact-polygons method
* Merge pull request `#209 <https://github.com/start-jsk/rtmros_tutorials/issues/209>`_ from mmurooka/add-jaxon-poose
  [hrpsys_ros_bridge_tutorials] add jaxon reset-manip-pose
* fixed pose name: init-pose2 -> collision-free-init-pose
* add jaxon reset-manip-pose, init-pose2
* [JAXON] fix jaxon.yaml
* Fix jaxon initial pose to avoid leg and hand collision
* Fix jaxon ee order (ee order = fsensor order)
* Add jaxon interface
* Add toe joint ee setting to hrpsys conf. Comment out by default
* update jaxon calib pose
* [hrpsys_ros_bridge_tutorials] Add HRP2JSK.urdf.xacro
* [hrpsys_ros_bridge_tutorials] Add multisense to HRP2JSK
* update limb order for jaxon
* fix joint order of shoulder
* [JAXON] update calibpose for eus model
* fix jaxon model
* fix jaxon model
* Merge pull request `#196 <https://github.com/start-jsk/rtmros_tutorials/issues/196>`_ from eisoku9618/fix-update_jaxon
  Fix update jaxon
* use BODY instead of WAIST
* recalculate with (print-end-effector-parameter-conf-from-robot *jaxon*)
* update conf parameter for JAXON
* [hrpsys_ros_bridge_tutorials] Add limb-controller to hrp2jsknt
* Merge pull request `#191 <https://github.com/start-jsk/rtmros_tutorials/issues/191>`_ from orikuma/modify-staro-robotiq-straight-end-coords
  Modify staro robotiq straight end coords
* [hrpsys_ros_bridge_tutorials] Run euslisp with disabling X when
  generating min-max table
* [hrpsys_ros_bridge_tutorials] Use multisense Stereo for HRP2JSKNT model
* [hrpsys_ros_bridge_tutorials] Fixed typo comment in staro.yaml
* [hrpsys_ros_bridge_tutorials] Remove unnecessary comment in staro.yaml
* [hrpsys_ros_bridge_tutorials] Modify straight version robotiq coords in staro.yaml
* [hrpsys_ros_bridge_tutorials] Modify right arm end-coords when robotiq is attached as straight
* [hrpsys_ros_bridge_tutorials] Modifiy sync-controller in staro-interface.l: use refernce-vector instead of potentio-vector
* [hrpsys_ros_bridge_tutorials] Add create-viewer option to staro-init
* [hrpsys_ros_bridge_tutorials] Remove objects function from staro-init
* [hrpsys_ros_bridge_tutorials] Update HRP2JSKNT model to be with multisense SL
* [hrpsys_ros_bridge_tutorials] Add robotiq model to staro robot-model and add options to make partial controller.
* [hrpsys_ros_bridge_tutorials] Add staro-interface.l and staro-utils.l
* [hrpsys_ros_bridge_tutorials] Use multisenseSL.urdf for HRP2JSKNTS
* Increase stride of test walk
* Add check program for walking commands
* [hrpsys_ros_bridge_tutorials] Check if jaxon_description is available
* update jaxon.yaml
* Update hrp2 robots dt;; 0.005->0.004[s]
* [hrpsys_ros_bridge_tutorials] Add jaxon_description to package.xml dependency
* [hrpsys_ros_bridge_tutorials] Add multisense_description and robotiq_description to
  build dependency
* [hrpsys_ros_bridge_tutorials] Remove gen_hand_attached_hrp2_model.sh and specify
  multiple links to remove to remove_sensor_from_urdf.py
* [hrpsys_ros_bridge_tutorial] Add suffix to temp file, because sometimes
  temp file names collide
* [hrpsys_ros_bridge_tutorials] Check multisense_description is available or not when generating
  HRP2JSKNTS model
* [hrpsys_ros_bridge_tutorials] Find hrpsys correctly because of updating of hrpsys build configuration
* [hrpsys_ros_bridge_tutorials] Add multisense model to HRP2JSKNTS
* [hrpsys_ros_bridge_tutorials] Use add_sensor_to_collada.py to generate
  sensor frames attached urdf files of hrp2s in order to remove stupid
  shell scripting.
* [hrpsys_ros_bridge_tutorials] add set(compile_robots ${compile_robots}
  PARENT_SCOPE) to make the variable global to prevent parallel execution
  of export_collada and rostest
* [hrpsys_ros_bridge_tutorials] Use euscollada/remove_sensor_from_urdf.py
  to remove link from urdf
* update jaxon.yaml
* [hrpsys_ros_bridge_tutorials] Fix path for catkin build
* update generating JAXON model (as same as STARO)
* update camera offset parameter for staro
* Fix joint order for hrp2w.yaml
* Add ystleg compile
* fix minor bug
* add xacro file for robotiq hand
* checkking existance of packages
* fix staro model for using multisense
* fix staro.yaml
* Add legs' crotch-y min max table for hrp2jsk robots
* Merge pull request `#154 <https://github.com/start-jsk/rtmros_tutorials/issues/154>`_ from YoheiKakiuchi/update_staro_model
  update staro model
* update staro model
* Make limit table to conf only if Euslisp min-max-table exists.
* Update latest hrpsys sample
* Add collision conf setting for samplerobot
* Merge pull request `#147 <https://github.com/start-jsk/rtmros_tutorials/issues/147>`_ from YoheiKakiuchi/fix_make
  正しい修正のようでしたのでMergeしました。
* Add minmax conf setting writing
* Add hrp4r util and set force-sensors from conf to include virtual force sensor
* Add hrp4r-interface.l. Currently auto-generated file. (We need to add :start-graps... and so on).
* change robot-init function to pass arguments to initializer
* fix makefile
* Update samplerobot reset-pose. Previous reset-pose occurs self collision. New reset-pose is moved from the initial line of OpenHRP-3.1/sample/controller/SampleController/etc/Sample.pos
* Add minmax table generation for hrp2w.l
* Merge pull request `#134 <https://github.com/start-jsk/rtmros_tutorials/issues/134>`_ from YoheiKakiuchi/add_jaxon_model
  add jaxon model
* Compile HRP4R in catkin system
* add jaxon model
* Add EUslisp version sample for rmfo param file
* Fix typo in gen_sensor_attached_hrp2_model.sh
* Add jig frame for calibration
* adding hrp2w-utils.l, including start and stop grasp
* adding vmax controller interface
* adding :start-grasp and :stop-grasp
* Generate urdf files with sensor frames
* Generate xacro handed models
* add collision_loop to STARO,URATALEG
* Revert abc_stride_parameter for backward compatibility according to https://github.com/start-jsk/rtmros_tutorials/issues/123#issuecomment-63620496
* Add sequence player sample and update sample function name
* Add unittest euslisp file for hrpsys-base sample
* (catkin, CMakeLists) : Fix SampleRobot end_coords setting
* update staro arms end-coords from contact coords to grasp coords
* (catkin, CMakeLists) : Remove deprecated AutBalancer stride_parameter conf setting
* (catkin, CMakeLists) : Set simulator time step for STARO and URATALEG as 0.002[s]
* add hrpsys_ros_bridge_tutorials dependency
* (CMakeLists, catkin.cmake) : Fix order of hrp2 End-effector.
* added script and launch files to publish end-effector tf
* (samplerobot-terrain-walk) : Update terrainwalk example to use rectanle and stair swing orbit mode.
* Merge branch 'master' of https://github.com/start-jsk/rtmros_tutorials into do-not-run-xacro-in-catkin
* do not run xacro when catkin_make.
* (hrp2w.yaml) : Update euslisp hrp2w reset-pose and add new sensor-calib pose
* (catkin.cmake, CMakeLists.txt) : Update hrp2w's conf setting
* fix using fullbody controller insted of leg controller
* add controller setting for each limb
* (hrpsys-samples) : Add Euslisp example corresponding to hrpsys-base/samples. Currently SampleRobot examples are added.
* (.rosinstall, manifest.xml) : Remove old dependency on jsk_recognition. These dependencies are already removed from package.xml for hydro environments
* (samplerobot-walk) : Use name instead of plist for footsteps
* (catkin.cmake, CMakeLists.txt) : Fix end-effector name (without colon) according to https://github.com/fkanehiro/hrpsys-base/pull/301
* update manip pose
* default end-coords : side version, commented-out-end-coords : straight version
* add test program for kf precision
* add macro for generating hand attached model to CMakeFile.txt. add current package to ROS_PACKAGE_PATH for xacro file
* generate HRP2JSKNT,NTS with hands
* overwrite :inverse-kinematics and :fullbody-inverse-kinematics for hrp2jsknt,nts not to use toe joint as default.
* (urataleg.yaml) : Update Urataleg reste-pose for more knee-bending pose
* (catkin.cmake) : Add testmdofarm compile for catkin
* fix end-coords of staro
* hrpsys_ros_bridge_tutorials/launch/samplerobot*.launch, hrpsys_ros_bridge_tutorials/CMakeLists.txt, hrpsys_ros_bridge_tutorials/catkin.cmake : use generated samplerobot*.launch instead committed files
* CMakeLists.txt, catkin.cmake : rename macro and update build of urataleg
* Merge pull request `#59 <https://github.com/start-jsk/rtmros_tutorials/issues/59>`_ from orikuma/add-staro-launch
  Add staro launch generation
* use unstable hrpsys_config
* fix argument passing for generation of launch and euslisp
* Added description to generate staro.launch for catkin_make
* Added description to generate staro.launch
* pass args to super class
* update openhrp dir path for euslisp and launch generation
* update angle-vector of reset-servo-off-pose in accordance with the change of sequence of angle-vector
* not use rosrun on catkin. it's not recommended
* do not generate "done file" under non existing directory and
  generate it under build the directory at the top level of the catkin workspace
* add message for else in openhrp3 compile
* update model path
* include and use common code for hrp2jsknt and hrp2jsknts
* include common code for hrp2jsknt and hrp2jsknts
* add existence check for vrml dir
* support catkin make
* add make joint min max table
* add hand servo methods
* add hand control methods to hrp2jsktns as well as hrp2jsknt
* update directories for closed JSK HRP2 robots
* check existence of handcontrol method
* load staro model from rbrain
* add hand model for hrp2jsknts
* remove tab for python yaml
* add handcontrol methods ;; controller codes and bridge codes are located in local repository
* add staro.yaml
* add urataleg and starto to catkin.cmake
* Merge remote-tracking branch 'origin/master' into add_staro
  Conflicts:
  hrpsys_ros_bridge_tutorials/CMakeLists.txt
* add launch file to run robot_pose_ekf
* fixed conf setting in catkin.cmake to become same with the setting in CMakeList.txt
* add urataleg collision pair
* add compiling urataleg on closed euslib directory
* fix hrp2 waist joint pitch and yaw alias in yaml
* remove DEPENDS openhrp3 hrpsys from catkin_package (`#31 <https://github.com/start-jsk/rtmros_tutorials/issues/31>`_)
* remove hrpsys catkin dependency
* add retry=4 for test code
* remove unset(openhrp3_LIBRARIES CACHE)
* Merge pull request `#22 <https://github.com/start-jsk/rtmros_tutorials/issues/22>`_ from k-okada/add_debug_message
  add debug message when openrhp3 is not found
* add debug message when openrhp3 is not found
* add parameters to conf file and interface.l for URATALEG
* add URATALEG to hrpsys_ros_bridge_tutorials
* tempolarily update HRP2JSKNT and HRP2JSKNTS end-coords setting according to https://github.com/start-jsk/rtmros_common/issues/379 ;; currently toe joints are not included
* remove deprecate conf setting for AutoBalancer RTC ;; update abc_leg_offset for HRP2 robots
* remove dependency to the libraries of hrpsys and openhrp3 from the cmake file
  generated by catkin.
* add STARO (copy from private repository)
* install with source permissions, and fix devel->install for all conf files
* Update package.xml
* Add rostest
* Add rostest
* fix conflict
* use pkg-config to set OPENHRP_SAMPLE_DIR
* add real robot walking parameter
* use rosdep for rviz
* add URATALEG to hrpsys_ros_bridge_tutorials
* fix conflict
* no need to make dependency to ALL, it's automatically generated in compile_openhrp_model
* add test code for hrpsys (check if generating dae,xml,conf are corret)
* fixing module name for openhrp3
* adding euscollada runtime dependency
* adding euscollada dependency
* adding dependency to euscollada
* Merge branch 'master' of https://github.com/garaemon/rtmros_tutorials
* adding rosdep dependency
* add dependency to hrpsys-base
* touch CMakeLists.txt to check travis
* add dependency to openhrp3
* fixing dependency
* adding dependency
* fix syntax errora around if(EXISTS sample1.wrl)
* fix for hoge/fuga check
* fix for hoge/fuga check
* check if pa10.main.wrl exists
* add rosdep names for rosdep install
* does not install .svn dir
* fix dependency
* depends on euscollada
* fix typoe hrpsys_SHARE -> HRPSYS_PREFIX
* use pkg_check_modules for openhrp3/hrpsys, and use hrpsys_PREFIX to work witho src and devel version
* set custom cmake file under CFG_EXTRAS, so that other package is abel to use macros defined in the cmake file
* add catkin_package to generate hrpsys_ros_brige_tutorials.cmake (hrpsys_ros_bridge_tutorials/catkin.cmake)
* install directory to the catkin install dir
* add hrp2 robots interface euslisp file
* catkinized hrpsys_ros_bridge_tutorials
* added grasp-pose to hrp3hand-util.l
* enable to change walk parameter
* add yaml file for testmdofarm
* add TESTMDOFARM.wrl
* add TESTMDOFARM robot
* use dump-seq-pattern-file function
* add utils for hrp3hand
* add utils file for hrp2jsknt ;; append hand
* add 3hand model compile
* add stair model which is generated from euslisp/jskeus/eus/models/darkgoldenrod-stairs-object.l
* add generate_default_launch_eusinterface_files for SampleRobot ;; currently comment outed
* enable to set PROJECT_FILE
* add samplerobot-walk3 and walk4 to use set-foot-steps
* add PDgains.sav and change CMakeLists.txt for using it
* add end_effector definition in conf ;; this will be merged with abc_end_effectors
* add end-effector setting for abc
* add HRP4R model compile as closed robots;; if model files do not exist, do not nothing
* rename compile_openhrp2_model -> compile_openhrp_model_for_closed_robots
* check existence of closed wrl directory
* use EUSTEST pkg path for euslisp interface and test file
* generate launch files for closed robots
* add hrp2 robot conversion ;; model files are not disclosed
* modify end-coords parent on SampleRobot
* add controller configuration for SampleRobot
* fix corba port to 15005, see Issue 141
* modify Makefile in hrpsys_ros_bridge_tutorials to make with catkinized compile_robot_model.cmake
* use 5005 port for rtls
* add pose-func for walk test and check adding of test-ros-init
* use -l option for rtls checking
* use -l option for rtls
* rename function names for hrpsys-ros-bridge test
* rename test program names
* add hrpsys-base and hrpsys-ros-bridge euslisp test for robots ;; currently not added to CMakeLists.txt's rosbuild_add_tests
* update to support NOSIM args
* re-organize rtmros_common, add openrtm_common, rtmros_tutorials, rtmros_hironx, rtmros_gazebo, openrtm_apps, See Issue 137
* Contributors: Hiroaki Yaguchi, Kei Okada, Kohei Kimura, Masaki Murooka, Ryohei Ueda, Shunichi Nozawa, Yohei Kakiuchi, Yu Ohara, Eisoku Kuroiwa, Iori Kumagai, Yuya Nagamatsu, Takuya Nakaokan, Ryo Terasawa
