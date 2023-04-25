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

# Generate HIRONXJSK_nosim.xml
find_package(hrp2_models QUIET)
if(${hrp2_models_FOUND})
  find_package(hrpsys_ros_bridge QUIET)
  if(${hrpsys_ros_bridge_FOUND})
    # Based on https://github.com/start-jsk/rtmros_hironx/blob/2.2.0/hironx_ros_bridge/CMakeLists.txt#L25-L48
    # set OPENHRP3 for configure_file
    set(PKG_CONFIG_PATH "${openhrp3_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}") # for openrtm3.1.pc
    execute_process(
      COMMAND pkg-config --variable=idl_dir openhrp3.1
      OUTPUT_VARIABLE OPENHRP_IDL_DIR
      RESULT_VARIABLE RESULT
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT RESULT EQUAL 0)
      set(OPENHRP_FOUND FALSE)
    endif()
    # OPENHRP_IDL_DIR = <openhrp3>/share/OpenHRP-3.1/idl/
    set(OPENHRP_SAMPLE_DIR ${OPENHRP_IDL_DIR}/../../../)
    set(OPENHRP3 ${OPENHRP_SAMPLE_DIR})  # for longfloor.wrl
    message("Configure HIRONXJSK_nosim.xml with OPENHRP3=${OPENHRP3}")
    configure_file(config/HIRONXJSK_nosim.xml.in ${PROJECT_SOURCE_DIR}/config/HIRONXJSK_nosim.xml)
    add_custom_target(${PROJECT_NAME}_model_files ALL DEPENDS ${PROJECT_SOURCE_DIR}/config/HIRONXJSK_nosim.xml)
  endif()
endif()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############
install(DIRECTORY config script worlds euslisp  models config launch scenes
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

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
