find_package(PkgConfig)
pkg_check_modules(omniorb omniORB4 REQUIRED)
pkg_check_modules(omnidynamic omniDynamic4 REQUIRED)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)
pkg_check_modules(openhrp3 openhrp3.1 REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)

list(GET hrpsys_INCLUDE_DIRS 0 hrpsys_first_incdir)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys)
list(APPEND hrpsys_INCLUDE_DIRS ${hrpsys_first_incdir}/hrpsys/idl)

include_directories(${catkin_INCLUDE_DIRS} ${openrtm_aist_INCLUDE_DIRS} ${openhrp3_INCLUDE_DIRS} ${hrpsys_INCLUDE_DIRS})
link_directories(${CATKIN_DEVEL_PREFIX}/lib ${hrpsys_PREFIX}/lib ${openhrp3_LIBRARY_DIRS} /opt/ros/$ENV{ROS_DISTRO}/lib/)

add_subdirectory(HRP3HandController)
