cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge_tutorials)

#find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys openhrp3)
find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge) # pr2_controllers_msgs robot_monitor

# find_package(PkgConfig)
# pkg_check_modules(openhrp3 openhrp3 REQUIRED)
# pkg_check_modules(hrpsys hrpsys-base REQUIRED)

catkin_package(
    DEPENDS #
    CATKIN-DEPENDS #
    INCLUDE_DIRS # TODO include
    LIBRARIES # TODO
)

if(EXISTS ${hrpsys_PREFIX}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl)
  compile_openhrp_model(
    ${hrpsys_PREFIX}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl
    HRP4C
    -a rightarm_torso,BODY,R_WRIST_R_LINK,0,0,0,0.707,0,0.707,0 -a leftarm_torso,BODY,L_WRIST_R_LINK,0,0,0,0.707,0,0.707,0 -a rightarm,BODY,CHEST_Y_LINK,0,0,0,0.707,0,0.707,0 -a leftarm,CHEST_Y_LINK,L_WRIST_R_LINK,0,0,0,0.707,0,0.707,0
    --conf-file-option "virtual_force_sensor: vlhsensor, CHEST_Y, L_HAND_J0, 0,0,0, 0,0,1,0, vrhsensor, CHEST_Y, R_HAND_J0, 0,0,0, 0,0,1,0"
    --conf-file-option "abc_leg_offset: 0.0, 0.06845, 0.0"
    --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
)
endif(EXISTS ${hrpsys_PREFIX}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl)

compile_openhrp_model(
  ${openhrp3_PREFIX}/share/openhrp3/share/OpenHRP-3.1/sample/model/PA10/pa10.main.wrl)
compile_openhrp_model(
  ${openhrp3_PREFIX}/share/openhrp3/share/OpenHRP-3.1/sample/model/sample1.wrl SampleRobot
  --conf-file-option "abc_leg_offset: 0,0.09,0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --conf-file-option "abc_end_effectors: :rarm,RARM_WRIST_P,CHEST, :larm,LARM_WRIST_P,CHEST, :rleg,RLEG_ANKLE_R,WAIST, :lleg,LLEG_ANKLE_R,WAIST"
  --conf-file-option "end_effectors: :rarm,RARM_WRIST_P,CHEST,0.0,-5.684342e-17,-0.12,9.813078e-18,1.0,0.0,1.5708, :larm,LARM_WRIST_P,CHEST,0.0,5.684342e-17,-0.12,-9.813078e-18,1.0,0.0,1.5708, :rleg,RLEG_ANKLE_R,WAIST,0.0,0.0,-0.07,0.0,0.0,0.0,0.0, :lleg,LLEG_ANKLE_R,WAIST,0.0,0.0,-0.07,0.0,0.0,0.0,0.0,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
)

# collada_robots
# webots_simulator
# yaskawa
# chreonoid

### convert model for closed models
macro(compile_openhrp_model_for_closed_robots _OpenHRP2_robot_vrml_name _OpenHRP2_robot_dir _OpenHRP2_robot_name)
  if(EXISTS $ENV{CVSDIR}/OpenHRP/etc/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl)
    compile_openhrp_model(
      $ENV{CVSDIR}/OpenHRP/etc/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl
      ${_OpenHRP2_robot_name}
      ${ARGN})
  endif()
endmacro()

# old HRP2xx.wrl files should be coverted.
compile_openhrp_model_for_closed_robots(HRP2JSK HRP2JSK_for_OpenHRP3 HRP2JSK
  --conf-file-option "abc_leg_offset: 0.0, 0.095, 0.0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --conf-file-option "abc_end_effectors: :rarm,RARM_JOINT6,CHEST_JOINT1, :larm,LARM_JOINT6,CHEST_JOINT1, :rleg,RLEG_JOINT5,WAIST, :lleg,LLEG_JOINT5,WAIST,"
  --conf-file-option "end_effectors: :rarm,RARM_JOINT6,CHEST_JOINT1,-5.684342e-17,0.0169,-0.174,-9.813078e-18,1.0,4.906539e-18,1.5708, :larm,LARM_JOINT6,CHEST_JOINT1,-5.684342e-17,-0.0169,-0.174,9.813078e-18,1.0,-4.906539e-18,1.5708, :rleg,RLEG_JOINT5,WAIST,0.0,-0.01,-0.105,0.0,0.0,0.0,0.0, :lleg,LLEG_JOINT5,WAIST,0.0,0.01,-0.105,0.0,0.0,0.0,0.0,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )

compile_openhrp_model_for_closed_robots(HRP2JSKNT HRP2JSKNT HRP2JSKNT
  --conf-file-option "abc_leg_offset: 0.0, 0.095, 0.0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --conf-file-option "abc_end_effectors: :rarm,RARM_JOINT6,CHEST_JOINT1, :larm,LARM_JOINT6,CHEST_JOINT1, :rleg,RLEG_JOINT5,WAIST, :lleg,LLEG_JOINT5,WAIST,"
  --conf-file-option "end_effectors: :rarm,RARM_JOINT6,CHEST_JOINT1,-0.0042,0.0392,-0.1245,-9.813078e-18,1.0,4.906539e-18,1.5708, :larm,LARM_JOINT6,CHEST_JOINT1,-0.0042,-0.0392,-0.1245,9.813078e-18,1.0,-4.906539e-18,1.5708, :rleg,RLEG_JOINT6,WAIST,-0.079411,-0.01,-0.031,0.0,0.0,0.0,0.0, :lleg,LLEG_JOINT6,WAIST,-0.079411,0.01,-0.031,0.0,0.0,0.0,0.0,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )
compile_openhrp_model_for_closed_robots(HRP2JSKNTS HRP2JSKNTS HRP2JSKNTS
  --conf-file-option "abc_leg_offset: 0.0, 0.095, 0.0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --conf-file-option "abc_end_effectors: :rarm,RARM_JOINT6,CHEST_JOINT1, :larm,LARM_JOINT6,CHEST_JOINT1, :rleg,RLEG_JOINT5,WAIST, :lleg,LLEG_JOINT5,WAIST,"
  --conf-file-option "end_effectors: :rarm,RARM_JOINT6,CHEST_JOINT1,-0.0042,0.0392,-0.1245,-9.813078e-18,1.0,4.906539e-18,1.5708, :larm,LARM_JOINT6,CHEST_JOINT1,-0.0042,-0.0392,-0.1245,9.813078e-18,1.0,-4.906539e-18,1.5708, :rleg,RLEG_JOINT6,WAIST,-0.079411,-0.01,-0.031,0.0,0.0,0.0,0.0, :lleg,LLEG_JOINT6,WAIST,-0.079411,0.01,-0.031,0.0,0.0,0.0,0.0,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )
# compile_openhrp_model_for_closed_robots(HRP2W HRP2W HRP2W)
# compile_openhrp_model_for_closed_robots(HRP2JSKNT HRP2JSKNT_WITH_3HAND HRP2JSKNT_WITH_3HAND
#  -a leftarm,CHEST_LINK1,LARM_LINK6,-0.0042,-0.0392,-0.1245,-3.373247e-18,1.0,9.813081e-18,1.5708,L_THUMBCM_Y,0,L_THUMBCM_P,1,L_INDEXMP_R,0,L_INDEXMP_P,0,L_INDEXPIP_R,-1,L_MIDDLEPIP_R,-1
#  -a leftarm_torso,BODY,LARM_LINK6,-0.0042,-0.0392,-0.1245,-3.373247e-18,1.0,9.813081e-18,1.5708,L_THUMBCM_Y,0,L_THUMBCM_P,1,L_INDEXMP_R,0,L_INDEXMP_P,0,L_INDEXPIP_R,-1,L_MIDDLEPIP_R,-1
#  -a leftarm_grasp,CHEST_LINK1,LARM_LINK6,0.0,-0.03,-0.17,1.0,0.0,0.0,2.0944,L_THUMBCM_Y,0,L_THUMBCM_P,1,L_INDEXMP_R,0,L_INDEXMP_P,0,L_INDEXPIP_R,-1,L_MIDDLEPIP_R,-1
#  -a rightarm,CHEST_LINK1,RARM_LINK6,-0.0042,0.0392,-0.1245,3.373247e-18,1.0,-9.813081e-18,1.5708,R_THUMBCM_Y,0,R_THUMBCM_P,1,R_INDEXMP_R,0,R_INDEXMP_P,0,R_INDEXPIP_R,1,R_MIDDLEPIP_R,1
#  -a rightarm_torso,BODY,RARM_LINK6,-0.0042,0.0392,-0.1245,3.373247e-18,1.0,-9.813081e-18,1.5708,R_THUMBCM_Y,0,R_THUMBCM_P,1,R_INDEXMP_R,0,R_INDEXMP_P,0,R_INDEXPIP_R,1,R_MIDDLEPIP_R,1
#  -a rightarm_grasp,CHEST_LINK1,RARM_LINK6,0.0,0.03,-0.17,-1.0,0.0,0.0,2.0944,R_THUMBCM_Y,0,R_THUMBCM_P,1,R_INDEXMP_R,0,R_INDEXMP_P,0,R_INDEXPIP_R,1,R_MIDDLEPIP_R,1
#   )
# compile_openhrp_model_for_closed_robots(HRP2JSKNTS HRP2JSKNTS_WITH_3HAND HRP2JSKNTS_WITH_3HAND
#   -a leftarm,CHEST_LINK1,LARM_LINK6,-0.0042,-0.0392,-0.1245,-3.373247e-18,1.0,9.813081e-18,1.5708,L_THUMBCM_Y,0,L_THUMBCM_P,1,L_INDEXMP_R,0,L_INDEXMP_P,0,L_INDEXPIP_R,-1,L_MIDDLEPIP_R,-1
#  -a leftarm_torso,BODY,LARM_LINK6,-0.0042,-0.0392,-0.1245,-3.373247e-18,1.0,9.813081e-18,1.5708,L_THUMBCM_Y,0,L_THUMBCM_P,1,L_INDEXMP_R,0,L_INDEXMP_P,0,L_INDEXPIP_R,-1,L_MIDDLEPIP_R,-1
#  -a leftarm_grasp,CHEST_LINK1,LARM_LINK6,0.0,-0.03,-0.17,1.0,0.0,0.0,2.0944,L_THUMBCM_Y,0,L_THUMBCM_P,1,L_INDEXMP_R,0,L_INDEXMP_P,0,L_INDEXPIP_R,-1,L_MIDDLEPIP_R,-1
#  -a rightarm,CHEST_LINK1,RARM_LINK6,-0.0042,0.0392,-0.1245,3.373247e-18,1.0,-9.813081e-18,1.5708,R_THUMBCM_Y,0,R_THUMBCM_P,1,R_INDEXMP_R,0,R_INDEXMP_P,0,R_INDEXPIP_R,1,R_MIDDLEPIP_R,1
#  -a rightarm_torso,BODY,RARM_LINK6,-0.0042,0.0392,-0.1245,3.373247e-18,1.0,-9.813081e-18,1.5708,R_THUMBCM_Y,0,R_THUMBCM_P,1,R_INDEXMP_R,0,R_INDEXMP_P,0,R_INDEXPIP_R,1,R_MIDDLEPIP_R,1
#  -a rightarm_grasp,CHEST_LINK1,RARM_LINK6,0.0,0.03,-0.17,-1.0,0.0,0.0,2.0944,R_THUMBCM_Y,0,R_THUMBCM_P,1,R_INDEXMP_R,0,R_INDEXMP_P,0,R_INDEXPIP_R,1,R_MIDDLEPIP_R,1
#   )
# compile_openhrp_model_for_closed_robots(HRP4R HRP4R HRP4R
#   -a leftarm,L_SHOULDER_P_LINK,L_WRIST_R_LINK,0,0,0,0,0,0,1,L_HAND_J0,-1,L_HAND_J1,-1
#   -a rightarm,R_SHOULDER_P_LINK,R_WRIST_R_LINK,0,0,0,0,0,0,1,R_HAND_J0,1,R_HAND_J1,1
#   --conf-file-option "virtual_force_sensor: vlhsensor, CHEST_Y, L_WRIST_R, 0,0,0, 0,0,1,0, vrhsensor, CHEST_Y, R_WRIST_R, 0,0,0, 0,0,1,0, vlfsensor, WAIST, L_ANKLE_R, 0,0,0, 0,0,1,0, vrfsensor, WAIST, R_ANKLE_R, 0,0,0, 0,0,1,0"
#   --conf-file-option "abc_leg_offset: 0.0, 0.079919, 0.0"
#   --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
#   --conf-file-option "abc_end_effectors: :rarm,R_WRIST_R,CHEST_Y, :larm,L_WRIST_R,CHEST_Y, :rleg,R_ANKLE_R,WAIST, :lleg,L_ANKLE_R,WAIST,"
#   --conf-file-option "end_effectors: :rarm,R_WRIST_R,CHEST_Y,0.0,0.0,-0.1,-1.471962e-17,1.0,-1.471962e-17,1.5708, :larm,L_WRIST_R,CHEST_Y,0.0,0.0,-0.1,1.471962e-17,1.0,1.471962e-17,1.5708, :rleg,R_ANKLE_R,WAIST,0.0,0.0,-0.091849,0.0,0.0,0.0,0.0, :lleg,L_ANKLE_R,WAIST,0.0,0.0,-0.091849,0.0,0.0,0.0,0.0,"
#   )

if(EXISTS $ENV{CVSDIR}/OpenHRP/etc/HRP3HAND_L/HRP3HAND_Lmain.wrl)
  compile_openhrp_model(
    $ENV{CVSDIR}/OpenHRP/etc/HRP3HAND_L/HRP3HAND_Lmain.wrl
    HRP3HAND_L)
endif()

if(EXISTS $ENV{CVSDIR}/OpenHRP/etc/HRP3HAND_R/HRP3HAND_Rmain.wrl)
  compile_openhrp_model(
    $ENV{CVSDIR}/OpenHRP/etc/HRP3HAND_R/HRP3HAND_Rmain.wrl
    HRP3HAND_R)
endif()

# kojiro

macro (generate_default_launch_eusinterface_files_for_jsk_hrpsys_ros_bridge_robots ROBOT_NAME)
  if(EXISTS $ENV{CVSDIR}/OpenHRP/etc/${ROBOT_NAME}/${ROBOT_NAME}main.wrl)
    generate_default_launch_eusinterface_files("$(env CVSDIR)/OpenHRP/etc/${ROBOT_NAME}/${ROBOT_NAME}main.wrl" hrpsys_ros_bridge_tutorials ${ROBOT_NAME} ${ARGV})
  endif()
endmacro ()
generate_default_launch_eusinterface_files_for_jsk_hrpsys_ros_bridge_robots(HRP2JSK "--no-euslisp")
generate_default_launch_eusinterface_files_for_jsk_hrpsys_ros_bridge_robots(HRP2JSKNT "--no-euslisp")
generate_default_launch_eusinterface_files_for_jsk_hrpsys_ros_bridge_robots(HRP2JSKNTS "--no-euslisp")
generate_default_launch_eusinterface_files_for_jsk_hrpsys_ros_bridge_robots(HRP2W "--no-euslisp")
generate_default_launch_eusinterface_files_for_jsk_hrpsys_ros_bridge_robots(HRP4R "--no-euslisp")

install(DIRECTORY euslisp launch scripts models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
