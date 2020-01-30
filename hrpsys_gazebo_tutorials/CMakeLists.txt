cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_tutorials)

find_package(catkin REQUIRED COMPONENTS euscollada hrpsys_ros_bridge hrpsys_ros_bridge_tutorials hrpsys_gazebo_general jsk_hrp2_ros_bridge hrp2_models euscollada xacro)

catkin_python_setup()

set(PKG_CONFIG_PATH ${hrpsys_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
pkg_check_modules(collada_urdf_jsk_patch collada_urdf_jsk_patch)

catkin_package(CATKIN_DEPENDS euscollada hrpsys_ros_bridge hrpsys_ros_bridge_tutorials hrpsys_gazebo_general)
  
add_subdirectory(rtc)

catkin_add_env_hooks(99.hrpsys_gazebo_tutorials SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

## install
install(DIRECTORY euslisp worlds launch config environment_models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)

if (jsk_hrp2_ros_bridge_SOURCE_PREFIX)
  set(jsk_hrp2_ros_bridge_PACKAGE_PATH ${jsk_hrp2_ros_bridge_SOURCE_PREFIX})
endif()
if (hrp2_models_SOURCE_PREFIX)
  set(hrp2_models_PACKAGE_PATH ${hrp2_models_SOURCE_PREFIX})
endif()
if (hrpsys_gazebo_general_SOURCE_PREFIX)
  set(hrpsys_gazebo_general_PACKAGE_PATH ${hrpsys_gazebo_general_SOURCE_PREFIX})
endif()

execute_process(COMMAND sed -e "s#@MODULE_LOAD_PATH@#${PROJECT_SOURCE_DIR}/lib,${hrpsys_gazebo_general_PACKAGE_PATH}/lib,${hrpsys_PREFIX}/lib#g" "${jsk_hrp2_ros_bridge_PACKAGE_PATH}/config/rtcdRobotModeHRP2Common.conf.in"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/config/rtcdRobotModeHRP2Common.conf)

execute_process(COMMAND sed -e "s#^model:.*$#model: ${hrp2_models_PACKAGE_PATH}/HRP2JSKNTS_for_OpenHRP3/HRP2JSKNTSmain.wrl#g" -e "s#^pdgains.file_name:.*$#pdgains.file_name: ${jsk_hrp2_ros_bridge_PACKAGE_PATH}/iob/PDgainsHRP2JSKNTS.sav#g" "${jsk_hrp2_ros_bridge_PACKAGE_PATH}/config/HRP2JSKNTS.conf"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/config/HRP2JSKNTS.conf)

execute_process(COMMAND sed -e "s#/home/grxuser/prog/rtmros_hrp2/jsk_hrp2_ros_bridge/force_sensor_calib_data/hand-calib_HRP2JSKNTS_[0-9]\\+#${PROJECT_SOURCE_DIR}/config/hand-calib_HRP2JSKNTS#g" -e "s#/home/grxuser/prog/rtmros_hrp2/jsk_hrp2_ros_bridge#${PROJECT_SOURCE_DIR}#g" -e "s#/home/grxuser/prog/rtmros_hrp2/hrp2_models#${hrp2_models_PACKAGE_PATH}#g" "${jsk_hrp2_ros_bridge_PACKAGE_PATH}/scripts/jsk_hrp2_setup.py"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/scripts/jsk_hrp2_gazebo_setup.py)

execute_process(COMMAND "${PROJECT_SOURCE_DIR}/scripts/replace_xmls.py" "${jsk_hrp2_ros_bridge_PACKAGE_PATH}/launch/hrp2017.launch" -C "${PROJECT_SOURCE_DIR}/config/hrp2017_gazebo.yaml"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/launch/hrp2017_gazebo.launch)

execute_process(COMMAND "${PROJECT_SOURCE_DIR}/scripts/replace_xmls.py" "${jsk_hrp2_ros_bridge_PACKAGE_PATH}/launch/hrp2017_ros_bridge.launch" -C "${PROJECT_SOURCE_DIR}/config/hrp2017_ros_bridge_gazebo.yaml"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/launch/hrp2017_ros_bridge_gazebo.launch)

execute_process(COMMAND "${PROJECT_SOURCE_DIR}/scripts/replace_xmls.py" "${jsk_hrp2_ros_bridge_PACKAGE_PATH}/launch/hrp2017_auditor.launch" -C "${PROJECT_SOURCE_DIR}/config/hrp2017_auditor_gazebo.yaml"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/launch/hrp2017_auditor_gazebo.launch)

execute_process(COMMAND "${xacro_PREFIX}/lib/xacro/xacro" "${PROJECT_SOURCE_DIR}/robot_models/HRP2JSKNTS/HRP2JSKNTS.urdf.xacro"
  OUTPUT_FILE ${PROJECT_SOURCE_DIR}/robot_models/HRP2JSKNTS/HRP2JSKNTS.urdf)
