cmake_minimum_required(VERSION 2.8.3)
project(hironx_tutorial)

find_package(catkin REQUIRED)

catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hironx_tutorial
#  CATKIN_DEPENDS
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Generate HIRONXJSK urdf for ros_control
find_package(hrpsys_ros_bridge_tutorials QUIET)
set(ORIG_URDF ${hrpsys_ros_bridge_tutorials_SOURCE_PREFIX}/models/HIRONXJSK.urdf)
set(MODEL_DIR ${PROJECT_SOURCE_DIR}/models)
set(OUTPUT_URDF ${MODEL_DIR}/HIRONXJSK.urdf)
if(${hrpsys_ros_bridge_tutorials_FOUND} AND EXISTS ${ORIG_URDF})
  add_custom_command(OUTPUT ${OUTPUT_URDF}
    COMMAND cp ${ORIG_URDF} ${OUTPUT_URDF} && patch -p1 < HIRONXJSK.patch
    WORKING_DIRECTORY ${MODEL_DIR}
    DEPENDS ${ORIG_URDF})
  add_custom_target(generate_hironxjsk_urdf ALL DEPENDS ${OUTPUT_URDF})
endif()

#############
## Install ##
#############
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
install(DIRECTORY scenes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
install(DIRECTORY script DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)
#install(DIRECTORY test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_hironx_tutorial.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()
#add_rostest(test/hironx_tutorial.test)

## Add folders to be run by python nosetests
#catkin_add_nosetests(test)
