cmake_minimum_required(VERSION 2.8.3)
project(hrpsys_ros_bridge_tutorials)

#find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge hrpsys openhrp3)
find_package(catkin REQUIRED COMPONENTS hrpsys_ros_bridge euscollada rostest euslisp)

set(PKG_CONFIG_PATH "${openhrp3_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}") # for openrtm3.1.pc
execute_process(
  COMMAND pkg-config --variable=idl_dir openhrp3.1
  OUTPUT_VARIABLE OPENHRP_IDL_DIR
  RESULT_VARIABLE RESULT
  OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT RESULT EQUAL 0)
  set(OPENHRP_FOUND FALSE)
  message("PKG_CONFIG_PATH = $ENV{PKG_CONFIG_PATH}")
  message(FATAL_ERROR "Could not found openhrp3.1")
endif()
set(OPENHRP_SAMPLE_DIR ${OPENHRP_IDL_DIR}/../sample)

if(NOT EXISTS ${hrpsys_ros_bridge_SOURCE_DIR}) # for installed package
  set(hrpsys_ros_bridge_SOURCE_DIR ${hrpsys_ros_bridge_PREFIX}/share/hrpsys_ros_bridge)
endif()

catkin_package(
    DEPENDS
    CATKIN_DEPENDS hrpsys_ros_bridge euscollada
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
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
)
endif(EXISTS ${hrpsys_PREFIX}/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl)

if(EXISTS ${OPENHRP_SAMPLE_DIR}/model/PA10/pa10.main.wrl)
compile_openhrp_model(
  ${OPENHRP_SAMPLE_DIR}/model/PA10/pa10.main.wrl)
endif(EXISTS ${OPENHRP_SAMPLE_DIR}/model/PA10/pa10.main.wrl)
if(EXISTS ${OPENHRP_SAMPLE_DIR}/model/sample1.wrl)
compile_openhrp_model(
  ${OPENHRP_SAMPLE_DIR}/model/sample1.wrl SampleRobot
  --conf-file-option "abc_leg_offset: 0,0.09,0"
  --conf-file-option "abc_stride_parameter: 0.15,0.05,10"
  --conf-file-option "end_effectors: lleg,LLEG_ANKLE_R,WAIST,0.0,0.0,-0.07,0.0,0.0,0.0,0.0, rleg,RLEG_ANKLE_R,WAIST,0.0,0.0,-0.07,0.0,0.0,0.0,0.0, larm,LARM_WRIST_P,CHEST,0.0,0,-0.12,0,1.0,0.0,1.5708, rarm,RARM_WRIST_P,CHEST,0.0,0,-0.12,0,1.0,0.0,1.5708,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
)
else()
  get_cmake_property(_variableNames VARIABLES)
  foreach (_variableName ${_variableNames})
    message(STATUS "${_variableName}=${${_variableName}}")
  endforeach()
  message(FATAL_ERROR "${OPENHRP_SAMPLE_DIR}/model/sample1.wrl not found")
endif(EXISTS ${OPENHRP_SAMPLE_DIR}/model/sample1.wrl)

# collada_robots
# webots_simulator
# yaskawa
# chreonoid

### convert model for closed models
macro(compile_model_for_closed_robots _robot_wrl_file _OpenHRP2_robot_name)
  if(EXISTS ${_robot_wrl_file})
    compile_openhrp_model(
      ${_robot_wrl_file}
      ${_OpenHRP2_robot_name}
      ${ARGN})
  else()
    message("\n\n\n\n ${_robot_wrl_file} is not found..\n\n\n\n")
  endif()
endmacro()
macro(compile_openhrp_model_for_closed_robots _OpenHRP2_robot_vrml_name _OpenHRP2_robot_dir _OpenHRP2_robot_name)
  compile_model_for_closed_robots(
    $ENV{CVSDIR}/OpenHRP/etc/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl
    ${_OpenHRP2_robot_name}
    ${ARGN})
endmacro()
macro(compile_rbrain_model_for_closed_robots _OpenHRP2_robot_vrml_name _OpenHRP2_robot_dir _OpenHRP2_robot_name)
  compile_model_for_closed_robots(
    $ENV{CVSDIR}/euslib/rbrain/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl
    ${_OpenHRP2_robot_name}
    ${ARGN})
endmacro()
macro(gen_minmax_table_for_closed_robots _OpenHRP2_robot_vrml_name _OpenHRP2_robot_dir _OpenHRP2_robot_name)
  if (EXISTS $ENV{CVSDIR}/OpenHRP/etc/${_OpenHRP2_robot_dir}/${_OpenHRP2_robot_vrml_name}main.wrl)
    string(TOLOWER ${_OpenHRP2_robot_name} _sname)
    set(_workdir ${PROJECT_SOURCE_DIR}/models)
    set(_gen_jointmm_command_arg "\"\\(write-min-max-table \\(${_sname}\\) \\\"${_workdir}/${_sname}.l\\\" :margin 1.0\\)\"")
    if(euslisp_SOURCE_DIR)
      set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_DIR})
    elseif(euslisp_SOURCE_PREFIX)
      set(euslisp_PACKAGE_PATH ${euslisp_SOURCE_PREFIX})
    else(euslisp_SOURCE_PREFIX)
      set(euslisp_PACKAGE_PATH ${euslisp_PREFIX}/share/euslisp)
    endif()

    execute_process(
      COMMAND find ${euslisp_PACKAGE_PATH} -name irteusgl -executable
      OUTPUT_VARIABLE irteusgl_path
      RESULT_VARIABLE result_
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT result_ EQUAL 0)
      message(FATAL_ERROR "failed to find euslisp, skipping generating min max table")
    else(NOT result_ EQUAL 0)
      set(euslisp_exe ${irteusgl_path})
      message("find euslisp on ${euslisp_exe}")
      add_custom_command(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done
      COMMAND ${euslisp_exe} ${PROJECT_SOURCE_DIR}/euslisp/make-joint-min-max-table.l ${_workdir}/${_sname}.l "\"${_gen_jointmm_command_arg}\"" "\"(exit)\"" && touch ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done
      DEPENDS ${_workdir}/${_sname}.l)
    add_custom_target(${_sname}_${PROJECT_NAME}_compile2 ALL DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_sname}_joint_minmax_done ${_sname}_${PROJECT_NAME}_compile)
    endif(NOT result_ EQUAL 0)
  endif()
endmacro()

# old HRP2xx.wrl files should be coverted.
compile_openhrp_model_for_closed_robots(HRP2JSK HRP2JSK_for_OpenHRP3 HRP2JSK
  --conf-file-option "abc_leg_offset: 0.0,0.105,0.0"
  --conf-file-option "end_effectors: rleg,RLEG_JOINT5,WAIST,0.0,-0.01,-0.105,0.0,0.0,0.0,0.0, lleg,LLEG_JOINT5,WAIST,0.0,0.01,-0.105,0.0,0.0,0.0,0.0, rarm,RARM_JOINT6,CHEST_JOINT1,0.0,0.0169,-0.174,0.0,1.0,0.0,1.5708, larm,LARM_JOINT6,CHEST_JOINT1,0.0,-0.0169,-0.174,0.0,1.0,0.0,1.5708,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )
gen_minmax_table_for_closed_robots(HRP2JSK HRP2JSK_for_OpenHRP3 HRP2JSK)

compile_openhrp_model_for_closed_robots(HRP2JSKNT HRP2JSKNT_for_OpenHRP3 HRP2JSKNT
  --conf-file-option "abc_leg_offset: 0.0,0.105,0.0"
  --conf-file-option "end_effectors: rleg,RLEG_JOINT5,WAIST,0.035589,-0.01,-0.105,0.0,0.0,0.0,0.0, lleg,LLEG_JOINT5,WAIST,0.035589,0.01,-0.105,0.0,0.0,0.0,0.0, rarm,RARM_JOINT6,CHEST_JOINT1,-0.0042,0.0392,-0.1245,0.0,1.0,0.0,1.5708, larm,LARM_JOINT6,CHEST_JOINT1,-0.0042,-0.0392,-0.1245,0.0,1.0,0.0,1.5708,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )
gen_minmax_table_for_closed_robots(HRP2JSKNT HRP2JSKNT_for_OpenHRP3 HRP2JSKNT)
compile_openhrp_model_for_closed_robots(HRP2JSKNTS HRP2JSKNTS_for_OpenHRP3 HRP2JSKNTS
  --conf-file-option "abc_leg_offset: 0.0,0.105,0.0"
  --conf-file-option "end_effectors: rleg,RLEG_JOINT5,WAIST,0.035589,-0.01,-0.105,0.0,0.0,0.0,0.0, lleg,LLEG_JOINT5,WAIST,0.035589,0.01,-0.105,0.0,0.0,0.0,0.0, rarm,RARM_JOINT6,CHEST_JOINT1,-0.0042,0.0392,-0.1245,0.0,1.0,0.0,1.5708, larm,LARM_JOINT6,CHEST_JOINT1,-0.0042,-0.0392,-0.1245,0.0,1.0,0.0,1.5708,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )
gen_minmax_table_for_closed_robots(HRP2JSKNTS HRP2JSKNTS_for_OpenHRP3 HRP2JSKNTS)
compile_openhrp_model_for_closed_robots(HRP2W HRP2W_for_OpenHRP3 HRP2W
  --conf-file-option "end_effectors: rarm,RARM_JOINT6,CHEST_JOINT1,-0.0042,0.0392,-0.1245,0.0,1.0,0.0,1.5708, larm,LARM_JOINT6,CHEST_JOINT1,-0.0042,-0.0392,-0.1245,0.0,1.0,0.0,1.5708,"
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  )
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
#   --conf-file-option "abc_end_effectors: rarm,R_WRIST_R,CHEST_Y, larm,L_WRIST_R,CHEST_Y, rleg,R_ANKLE_R,WAIST, lleg,L_ANKLE_R,WAIST,"
#   --conf-file-option "end_effectors: rarm,R_WRIST_R,CHEST_Y,0.0,0.0,-0.1,-1.471962e-17,1.0,-1.471962e-17,1.5708, larm,L_WRIST_R,CHEST_Y,0.0,0.0,-0.1,1.471962e-17,1.0,1.471962e-17,1.5708, rleg,R_ANKLE_R,WAIST,0.0,0.0,-0.091849,0.0,0.0,0.0,0.0, lleg,L_ANKLE_R,WAIST,0.0,0.0,-0.091849,0.0,0.0,0.0,0.0,"
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

# URATALEG
compile_rbrain_model_for_closed_robots(URATALEG urataleg URATALEG
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  --conf-file-option "abc_leg_offset: 0.0, 0.08, 0.0"
  --conf-file-option "end_effectors: rleg,RLEG_JOINT5,WAIST,0.0,0.0,-0.096,0.0,0.0,0.0,0.0, lleg,LLEG_JOINT5,WAIST,0.0,0.0,-0.096,0.0,0.0,0.0,0.0,"
  --conf-dt-option "0.002"
  --simulation-timestep-option "0.002"
#  --conf-file-option "collision_pair: WAIST:LLEG_JOINT1 WAIST:LLEG_JOINT2 WAIST:LLEG_JOINT3 WAIST:LLEG_JOINT4 WAIST:LLEG_JOINT5 WAIST:RLEG_JOINT1 WAIST:RLEG_JOINT2 WAIST:RLEG_JOINT3 WAIST:RLEG_JOINT4 WAIST:RLEG_JOINT5 LLEG_JOINT0:RLEG_JOINT0 LLEG_JOINT0:LLEG_JOINT2 LLEG_JOINT0:LLEG_JOINT3 LLEG_JOINT0:LLEG_JOINT4 LLEG_JOINT0:LLEG_JOINT5 LLEG_JOINT0:RLEG_JOINT1 LLEG_JOINT0:RLEG_JOINT2 LLEG_JOINT0:RLEG_JOINT3 LLEG_JOINT0:RLEG_JOINT4 LLEG_JOINT0:RLEG_JOINT5 RLEG_JOINT0:LLEG_JOINT1 RLEG_JOINT0:LLEG_JOINT2 RLEG_JOINT0:LLEG_JOINT3 RLEG_JOINT0:LLEG_JOINT4 RLEG_JOINT0:LLEG_JOINT5 RLEG_JOINT0:RLEG_JOINT2 RLEG_JOINT0:RLEG_JOINT3 RLEG_JOINT0:RLEG_JOINT4 RLEG_JOINT0:RLEG_JOINT5 LLEG_JOINT1:LLEG_JOINT3 LLEG_JOINT1:LLEG_JOINT4 LLEG_JOINT1:LLEG_JOINT5 LLEG_JOINT1:RLEG_JOINT1 LLEG_JOINT1:RLEG_JOINT2 LLEG_JOINT1:RLEG_JOINT3 LLEG_JOINT1:RLEG_JOINT4 LLEG_JOINT1:RLEG_JOINT5 LLEG_JOINT2:LLEG_JOINT4 LLEG_JOINT2:LLEG_JOINT5 LLEG_JOINT2:RLEG_JOINT1 LLEG_JOINT2:RLEG_JOINT2 LLEG_JOINT2:RLEG_JOINT3 LLEG_JOINT2:RLEG_JOINT4 LLEG_JOINT2:RLEG_JOINT5 LLEG_JOINT3:LLEG_JOINT5 LLEG_JOINT3:RLEG_JOINT1 LLEG_JOINT3:RLEG_JOINT2 LLEG_JOINT3:RLEG_JOINT3 LLEG_JOINT3:RLEG_JOINT4 LLEG_JOINT3:RLEG_JOINT5 LLEG_JOINT4:RLEG_JOINT1 LLEG_JOINT4:RLEG_JOINT2 LLEG_JOINT4:RLEG_JOINT3 LLEG_JOINT4:RLEG_JOINT4 LLEG_JOINT4:RLEG_JOINT5 LLEG_JOINT5:RLEG_JOINT1 LLEG_JOINT5:RLEG_JOINT2 LLEG_JOINT5:RLEG_JOINT3 LLEG_JOINT5:RLEG_JOINT4 LLEG_JOINT5:RLEG_JOINT5 RLEG_JOINT1:RLEG_JOINT3 RLEG_JOINT1:RLEG_JOINT4 RLEG_JOINT1:RLEG_JOINT5 RLEG_JOINT2:RLEG_JOINT4 RLEG_JOINT2:RLEG_JOINT5 RLEG_JOINT3:RLEG_JOINT5"
  --conf-file-option "collision_pair: WAIST:LLEG_JOINT1 WAIST:LLEG_JOINT3 WAIST:LLEG_JOINT4 WAIST:RLEG_JOINT1 WAIST:RLEG_JOINT2 WAIST:RLEG_JOINT3 WAIST:RLEG_JOINT4 WAIST:RLEG_JOINT5 LLEG_JOINT0:RLEG_JOINT0 LLEG_JOINT0:LLEG_JOINT3 LLEG_JOINT0:LLEG_JOINT4 LLEG_JOINT0:LLEG_JOINT5 LLEG_JOINT0:RLEG_JOINT1 LLEG_JOINT0:RLEG_JOINT2 LLEG_JOINT0:RLEG_JOINT3 LLEG_JOINT0:RLEG_JOINT4 LLEG_JOINT0:RLEG_JOINT5 RLEG_JOINT0:LLEG_JOINT1 RLEG_JOINT0:LLEG_JOINT2 RLEG_JOINT0:LLEG_JOINT3 RLEG_JOINT0:LLEG_JOINT4 RLEG_JOINT0:LLEG_JOINT5 RLEG_JOINT0:RLEG_JOINT3 RLEG_JOINT0:RLEG_JOINT4 RLEG_JOINT0:RLEG_JOINT5 LLEG_JOINT1:LLEG_JOINT3 LLEG_JOINT1:LLEG_JOINT4 LLEG_JOINT1:LLEG_JOINT5 LLEG_JOINT1:RLEG_JOINT1 LLEG_JOINT1:RLEG_JOINT2 LLEG_JOINT1:RLEG_JOINT3 LLEG_JOINT1:RLEG_JOINT4 LLEG_JOINT1:RLEG_JOINT5 LLEG_JOINT2:LLEG_JOINT4 LLEG_JOINT2:LLEG_JOINT5 LLEG_JOINT2:RLEG_JOINT1 LLEG_JOINT2:RLEG_JOINT2 LLEG_JOINT2:RLEG_JOINT3 LLEG_JOINT2:RLEG_JOINT4 LLEG_JOINT2:RLEG_JOINT5 LLEG_JOINT3:RLEG_JOINT1 LLEG_JOINT3:RLEG_JOINT2 LLEG_JOINT3:RLEG_JOINT3 LLEG_JOINT3:RLEG_JOINT4 LLEG_JOINT3:RLEG_JOINT5 LLEG_JOINT4:RLEG_JOINT1 LLEG_JOINT4:RLEG_JOINT2 LLEG_JOINT4:RLEG_JOINT3 LLEG_JOINT4:RLEG_JOINT4 LLEG_JOINT4:RLEG_JOINT5 LLEG_JOINT5:RLEG_JOINT1 LLEG_JOINT5:RLEG_JOINT2 LLEG_JOINT5:RLEG_JOINT3 LLEG_JOINT5:RLEG_JOINT4 LLEG_JOINT5:RLEG_JOINT5 RLEG_JOINT1:RLEG_JOINT3 RLEG_JOINT1:RLEG_JOINT4 RLEG_JOINT1:RLEG_JOINT5 RLEG_JOINT2:RLEG_JOINT4 RLEG_JOINT2:RLEG_JOINT5"
  )

# STARO
compile_rbrain_model_for_closed_robots(STARO staro STARO
  --robothardware-conf-file-option "pdgains.file_name: ${PROJECT_SOURCE_DIR}/models/PDgains.sav"
  --conf-file-option "abc_leg_offset: 0.0, 0.1, 0.0"
  --conf-file-option "end_effectors: rarm,RARM_JOINT7,CHEST_JOINT1,0.0,-0.15701,0.0,0.57735,-0.57735,-0.57735,2.0944, larm,LARM_JOINT7,CHEST_JOINT1,-5.684342e-17,0.15701,-1.136868e-16,-0.57735,-0.57735,0.57735,2.0944, rleg,RLEG_JOINT5,WAIST,0.0,0.0,-0.096,0.0,0.0,0.0,0.0, lleg,LLEG_JOINT5,WAIST,0.0,0.0,-0.096,0.0,0.0,0.0,0.0,"
  --conf-file-option "torque_offset: 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0"
  --conf-file-option "torque_filter_params: 2, 1.0, 1.88903, -0.89487, 0.0014603, 0.0029206, 0.0014603"
  --conf-file-option "error_to_torque_gain: 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0"
  --conf-file-option "error_dead_zone: 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0,   0, 0, 0, 0, 0, 0,  0, 0, 0, 0, 0, 0"
  --conf-file-option "torque_gain: 0.0001"
  --conf-file-option "collision_pair: WAIST:RARM_JOINT2 WAIST:LARM_JOINT2 RLEG_JOINT2:RLEG_JOINT5 LLEG_JOINT2:LLEG_JOINT5 RARM_JOINT7:LARM_JOINT7 RARM_JOINT6:LARM_JOINT7 RARM_JOINT7:LARM_JOINT6 LARM_JOINT7:RLEG_JOINT5 RARM_JOINT7:RLEG_JOINT5 RARM_JOINT5:LARM_JOINT7 RARM_JOINT7:LARM_JOINT5 RARM_JOINT7:LLEG_JOINT5 LARM_JOINT7:LLEG_JOINT5 RARM_JOINT6:LARM_JOINT6 RARM_JOINT7:RLEG_JOINT4 LARM_JOINT7:RLEG_JOINT4 RARM_JOINT7:LLEG_JOINT4 LARM_JOINT6:LLEG_JOINT5 RARM_JOINT6:LARM_JOINT5 LARM_JOINT6:RLEG_JOINT5 RARM_JOINT4:LARM_JOINT7 RARM_JOINT7:LARM_JOINT4 RARM_JOINT6:LLEG_JOINT5 LARM_JOINT7:LLEG_JOINT4 RARM_JOINT5:LARM_JOINT6 RARM_JOINT6:RLEG_JOINT5 RARM_JOINT4:LARM_JOINT6 RARM_JOINT7:RLEG_JOINT3 LARM_JOINT7:LLEG_JOINT3 RARM_JOINT7:LLEG_JOINT3 RARM_JOINT7:LARM_JOINT3 RARM_JOINT5:LARM_JOINT5 LARM_JOINT7:RLEG_JOINT3 RARM_JOINT6:LLEG_JOINT4 LARM_JOINT6:LLEG_JOINT4 RARM_JOINT6:RLEG_JOINT4 RLEG_JOINT5:LLEG_JOINT5 LARM_JOINT5:RLEG_JOINT5 RARM_JOINT5:RLEG_JOINT5 RARM_JOINT5:LLEG_JOINT5 RARM_JOINT3:LARM_JOINT7 LARM_JOINT5:LLEG_JOINT5 RARM_JOINT6:LARM_JOINT4 LARM_JOINT6:LLEG_JOINT3 RLEG_JOINT5:LLEG_JOINT4 LARM_JOINT6:RLEG_JOINT3 LARM_JOINT5:RLEG_JOINT4 RARM_JOINT6:LLEG_JOINT3 RARM_JOINT5:RLEG_JOINT4 RARM_JOINT2:LARM_JOINT7 RARM_JOINT5:LARM_JOINT4 LARM_JOINT4:LLEG_JOINT5 RARM_JOINT6:RLEG_JOINT3 RARM_JOINT7:LARM_JOINT2 LARM_JOINT4:RLEG_JOINT5 RARM_JOINT5:LLEG_JOINT4 RLEG_JOINT4:LLEG_JOINT5 RARM_JOINT4:RLEG_JOINT5 RARM_JOINT7:RLEG_JOINT2 RARM_JOINT6:LARM_JOINT3 LARM_JOINT7:RLEG_JOINT2 LARM_JOINT5:LLEG_JOINT4 RARM_JOINT4:LLEG_JOINT5 RARM_JOINT4:LARM_JOINT5 RARM_JOINT3:LARM_JOINT6 LARM_JOINT7:LLEG_JOINT2 RARM_JOINT7:LLEG_JOINT2 RARM_JOINT3:LLEG_JOINT5 RARM_JOINT3:LARM_JOINT5 RARM_JOINT6:RLEG_JOINT2 RARM_JOINT5:LLEG_JOINT3 RARM_JOINT3:RLEG_JOINT5 RARM_JOINT5:RLEG_JOINT3 RARM_JOINT6:LLEG_JOINT2 LARM_JOINT5:RLEG_JOINT3 RLEG_JOINT5:LLEG_JOINT3 LARM_JOINT4:LLEG_JOINT4 LARM_JOINT4:RLEG_JOINT4 RLEG_JOINT4:LLEG_JOINT4 LARM_JOINT3:LLEG_JOINT5 LARM_JOINT5:LLEG_JOINT3 LARM_JOINT3:RLEG_JOINT5 RLEG_JOINT3:LLEG_JOINT5 RARM_JOINT5:LARM_JOINT3 LARM_JOINT6:RLEG_JOINT2 LARM_JOINT6:LLEG_JOINT2 RARM_JOINT1:LARM_JOINT7 RARM_JOINT7:LARM_JOINT1 RARM_JOINT5:RLEG_JOINT2 LARM_JOINT5:LLEG_JOINT2 RARM_JOINT4:LLEG_JOINT3 LARM_JOINT5:RLEG_JOINT2 LARM_JOINT4:LLEG_JOINT3 LARM_JOINT4:RLEG_JOINT3 LARM_JOINT3:LLEG_JOINT4 LARM_JOINT3:RLEG_JOINT4 LARM_JOINT2:LLEG_JOINT5 LARM_JOINT2:RLEG_JOINT5 RARM_JOINT5:LLEG_JOINT2 LARM_JOINT7:RLEG_JOINT0 RARM_JOINT7:LLEG_JOINT0 RARM_JOINT7:RLEG_JOINT0 RARM_JOINT2:RLEG_JOINT5 RARM_JOINT2:LLEG_JOINT5 RLEG_JOINT2:LLEG_JOINT5 RLEG_JOINT3:LLEG_JOINT4 RARM_JOINT3:RLEG_JOINT4 LARM_JOINT7:LLEG_JOINT0 RARM_JOINT3:LLEG_JOINT4 RLEG_JOINT4:LLEG_JOINT3 RARM_JOINT4:RLEG_JOINT3 RLEG_JOINT5:LLEG_JOINT2 RARM_JOINT1:LLEG_JOINT5 WAIST:RARM_JOINT7 RLEG_JOINT1:LLEG_JOINT5 RARM_JOINT1:RLEG_JOINT5 RLEG_JOINT2:LLEG_JOINT4 LARM_JOINT1:RLEG_JOINT5 RLEG_JOINT5:LLEG_JOINT1 LARM_JOINT6:LLEG_JOINT0 LARM_JOINT1:LLEG_JOINT5 RLEG_JOINT3:LLEG_JOINT3 LARM_JOINT4:LLEG_JOINT2 WAIST:LARM_JOINT7 RARM_JOINT4:LLEG_JOINT2 LARM_JOINT4:RLEG_JOINT2 LARM_JOINT3:RLEG_JOINT3 RLEG_JOINT4:LLEG_JOINT2 LARM_JOINT3:LLEG_JOINT3 RARM_JOINT6:RLEG_JOINT0 RARM_JOINT4:RLEG_JOINT2 RARM_JOINT3:RLEG_JOINT3 RARM_JOINT2:RLEG_JOINT4 RARM_JOINT6:LLEG_JOINT0 RARM_JOINT3:LLEG_JOINT3 RARM_JOINT0:RARM_JOINT7 LARM_JOINT2:RLEG_JOINT3 LARM_JOINT0:LLEG_JOINT5 RLEG_JOINT3:LLEG_JOINT2 LARM_JOINT3:RLEG_JOINT2 RARM_JOINT0:RLEG_JOINT5 RARM_JOINT5:RLEG_JOINT0 LARM_JOINT3:LLEG_JOINT2 LARM_JOINT2:LLEG_JOINT3 RARM_JOINT3:RLEG_JOINT2 RARM_JOINT3:LLEG_JOINT2 RARM_JOINT5:LLEG_JOINT0 RLEG_JOINT4:LLEG_JOINT1 RLEG_JOINT0:LLEG_JOINT5 WAIST:RARM_JOINT6 LARM_JOINT5:RLEG_JOINT0 RARM_JOINT1:RLEG_JOINT4 WAIST:LARM_JOINT6 RLEG_JOINT2:LLEG_JOINT3 LARM_JOINT0:LARM_JOINT7 RARM_JOINT2:RLEG_JOINT3 LARM_JOINT5:LLEG_JOINT0 RLEG_JOINT5:LLEG_JOINT0 RARM_JOINT0:RARM_JOINT6 WAIST:RARM_JOINT5 WAIST:RLEG_JOINT5 WAIST:LARM_JOINT5 WAIST:LLEG_JOINT5 LARM_JOINT4:LLEG_JOINT0 RARM_JOINT4:RLEG_JOINT0 RLEG_JOINT0:LLEG_JOINT4 RLEG_JOINT1:LLEG_JOINT3 RARM_JOINT1:RLEG_JOINT3 RLEG_JOINT2:LLEG_JOINT2 LARM_JOINT0:LARM_JOINT6 RARM_JOINT1:RARM_JOINT7 RLEG_JOINT4:LLEG_JOINT0 LARM_JOINT2:LLEG_JOINT2 LARM_JOINT2:RLEG_JOINT2 RARM_JOINT0:RLEG_JOINT4 LARM_JOINT1:LLEG_JOINT3 LARM_JOINT1:RLEG_JOINT3 RLEG_JOINT3:LLEG_JOINT1 LARM_JOINT1:LARM_JOINT7 RLEG_JOINT2:LLEG_JOINT1 RLEG_JOINT1:LLEG_JOINT2 RLEG_JOINT3:LLEG_JOINT0 RLEG_JOINT0:LLEG_JOINT3 RLEG_JOINT0:RLEG_JOINT5 LLEG_JOINT0:LLEG_JOINT5 RARM_JOINT0:RLEG_JOINT3 WAIST:LLEG_JOINT4 LARM_JOINT0:LARM_JOINT5 RARM_JOINT3:RLEG_JOINT0 RARM_JOINT0:RARM_JOINT5 LARM_JOINT3:LLEG_JOINT0 RARM_JOINT0:LLEG_JOINT3 RARM_JOINT2:RARM_JOINT7 RARM_JOINT1:RARM_JOINT6 RARM_JOINT1:RLEG_JOINT2 LARM_JOINT1:LLEG_JOINT2 WAIST:RARM_JOINT4 LARM_JOINT2:LARM_JOINT7 LARM_JOINT0:RLEG_JOINT3 WAIST:LARM_JOINT4 LARM_JOINT1:RLEG_JOINT2 LARM_JOINT1:LARM_JOINT6 WAIST:RLEG_JOINT4 LARM_JOINT0:LLEG_JOINT3 WAIST:RLEG_JOINT3 RARM_JOINT0:RLEG_JOINT2 WAIST:LARM_JOINT3 RLEG_JOINT1:RLEG_JOINT5 RARM_JOINT1:RARM_JOINT5 WAIST:RARM_JOINT3 WAIST:LLEG_JOINT3 LARM_JOINT3:LARM_JOINT7 LARM_JOINT1:LARM_JOINT5 LARM_JOINT0:LLEG_JOINT2 LARM_JOINT0:RLEG_JOINT2 RARM_JOINT3:RARM_JOINT7 RLEG_JOINT0:LLEG_JOINT2 LLEG_JOINT1:LLEG_JOINT5 RLEG_JOINT2:LLEG_JOINT0 WAIST:RARM_JOINT2 LLEG_JOINT2:LLEG_JOINT5 RLEG_JOINT2:RLEG_JOINT5 WAIST:LARM_JOINT2"
  --conf-dt-option "0.002"
  --simulation-timestep-option "0.002"
  )

if(EXISTS ${PROJECT_SOURCE_DIR}/models/TESTMDOFARM.wrl)
  compile_openhrp_model(
    ${PROJECT_SOURCE_DIR}/models/TESTMDOFARM.wrl
    TESTMDOFARM)
endif()

macro (generate_default_launch_eusinterface_files_for_jsk_closed_openhrp_robots ROBOT_DIR ROBOT_NAME)
  set(_arg_list ${ARGV})
  # remove arguments of this macro
  list(REMOVE_AT _arg_list 0 1)
  if(EXISTS $ENV{CVSDIR}/OpenHRP/etc/${ROBOT_DIR}/${ROBOT_NAME}main.wrl)
    generate_default_launch_eusinterface_files("$(env CVSDIR)/OpenHRP/etc/${ROBOT_DIR}/${ROBOT_NAME}main.wrl" hrpsys_ros_bridge_tutorials ${ROBOT_NAME} ${_arg_list})
  endif()
endmacro ()
macro (generate_default_launch_eusinterface_files_for_jsk_closed_rbrain_robots ROBOT_DIR ROBOT_NAME)
  set(_arg_list ${ARGV})
  # remove arguments of this macro
  list(REMOVE_AT _arg_list 0 1)
  if(EXISTS $ENV{CVSDIR}/euslib/rbrain/${ROBOT_DIR}/${ROBOT_NAME}main.wrl)
    generate_default_launch_eusinterface_files("$(env CVSDIR)/euslib/rbrain/${ROBOT_DIR}/${ROBOT_NAME}main.wrl" hrpsys_ros_bridge_tutorials ${ROBOT_NAME} ${_arg_list})
  endif()
endmacro ()

generate_default_launch_eusinterface_files_for_jsk_closed_openhrp_robots(HRP2JSK_for_OpenHRP3 HRP2JSK "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files_for_jsk_closed_openhrp_robots(HRP2JSKNT_for_OpenHRP3 HRP2JSKNT "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files_for_jsk_closed_openhrp_robots(HRP2JSKNTS_for_OpenHRP3 HRP2JSKNTS "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files_for_jsk_closed_openhrp_robots(HRP2W_for_OpenHRP3 HRP2W "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files_for_jsk_closed_openhrp_robots(HRP4R HRP4R "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files_for_jsk_closed_rbrain_robots(staro STARO "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files_for_jsk_closed_rbrain_robots(urataleg URATALEG "--use-unstable-hrpsys-config")
generate_default_launch_eusinterface_files(
  "$(find hrpsys_ros_bridge_tutorials)/models/TESTMDOFARM.wrl"
  hrpsys_ros_bridge_tutorials
  TESTMDOFARM
  "--no-euslisp")
generate_default_launch_eusinterface_files("$(find openhrp3)/share/OpenHRP-3.1/sample/model/sample1.wrl" hrpsys_ros_bridge_tutorials SampleRobot "--use-unstable-hrpsys-config")

macro (generate_hand_attached_hrp2_model _robot_name)
  set(_model_dir "${PROJECT_SOURCE_DIR}/models/")
  set(_in_urdf_file "${_model_dir}/${_robot_name}.urdf")
  set(_out_urdf_file "${_model_dir}/${_robot_name}_body.urdf")
  string(TOLOWER ${_robot_name} _srobot_name)
  set(_script_file "${PROJECT_SOURCE_DIR}/models/gen_hand_attached_hrp2_model.sh")
  message("generate hand_attached_hrp2_model for ${_robot_name}")
  add_custom_command(OUTPUT ${_out_urdf_file}
      COMMAND ${_script_file} ${_robot_name} ${_in_urdf_file} ${PROJECT_SOURCE_DIR}/..
      DEPENDS ${_in_urdf_file})
  add_custom_target(${_robot_name}_model_generate DEPENDS ${_out_urdf_file})
  list(APPEND compile_urdf_robots ${_robot_name}_model_generate)
endmacro()

if(EXISTS $ENV{CVSDIR}/OpenHRP/etc/HRP3HAND_R/HRP3HAND_Rmain.wrl)
  generate_hand_attached_hrp2_model(HRP2JSKNT)
  generate_hand_attached_hrp2_model(HRP2JSKNTS)
  add_custom_target(all_robots_model_generate ALL DEPENDS ${compile_urdf_robots})
endif()

install(DIRECTORY euslisp launch scripts models test DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION} USE_SOURCE_PERMISSIONS)
install(CODE
  "execute_process(COMMAND echo \"fix \$ENV{DESTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/model/* ${CATKIN_DEVEL_PREFIX} -> ${CMAKE_INSTALL_PREFIX}\")
   file(GLOB _conf_files \"\$ENV{DISTDIR}/${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/model/*/*.conf\")
   foreach(_conf_file \${_conf_files})
     execute_process(COMMAND sed -i s@${CATKIN_DEVEL_PREFIX}@${CMAKE_INSTALL_PREFIX}@g \${_conf_file})
     execute_process(COMMAND sed -i s@${hrpsys_ros_bridge_tutorials_SOURCE_DIR}@${CMAKE_INSTALL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}@g \${_conf_file})
  endforeach()
")

add_rostest(test/test_hrpsys_pa10.launch)
add_rostest(test/test_hrpsys_samplerobot.launch)
