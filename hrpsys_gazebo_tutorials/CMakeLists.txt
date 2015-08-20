cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_gazebo_tutorials)

find_package(catkin REQUIRED COMPONENTS euscollada hrpsys_ros_bridge hrpsys_ros_bridge_tutorials hrpsys_gazebo_general)

set(PKG_CONFIG_PATH ${hrpsys_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH})
find_package(PkgConfig)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
pkg_check_modules(collada_urdf_jsk_patch collada_urdf_jsk_patch)

catkin_package(CATKIN_DEPENDS euscollada hrpsys_ros_bridge hrpsys_ros_bridge_tutorials hrpsys_gazebo_general)

catkin_add_env_hooks(99.hrpsys_gazebo_tutorials SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)

## install
install(DIRECTORY euslisp worlds launch config environment_models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(DIRECTORY scripts DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
