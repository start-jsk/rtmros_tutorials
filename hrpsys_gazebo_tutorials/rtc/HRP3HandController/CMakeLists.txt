if(EXISTS ${hrpsys_SOURCE_DIR})
  set(SequencePlayerDirectory ${hrpsys_SOURCE_DIR}/src/rtc/SequencePlayer)
elseif(EXISTS ${hrpsys_SOURCE_PREFIX})
  set(SequencePlayerDirectory ${hrpsys_SOURCE_PREFIX}/src/rtc/SequencePlayer)
else()
  set(SequencePlayerDirectory ${hrpsys_PREFIX}/share/hrpsys/src/rtc/SequencePlayer)
endif()

if(EXISTS ${hrpsys_gazebo_general_SOURCE_DIR})
  set(iobDirectory ${hrpsys_gazebo_general_SOURCE_DIR}/iob)
else(EXISTS ${hrpsys_gazebo_general_SOURCE_PREFIX})
  set(iobDirectory ${hrpsys_gazebo_general_SOURCE_PREFIX}/iob)
endif()

find_package(jsk_hrp2_ros_bridge)
list(GET jsk_hrp2_ros_bridge_INCLUDE_DIRS 0 jsk_hrp2_ros_bridge_first_incdir)
list(APPEND jsk_hrp2_ros_bridge_INCLUDE_DIRS ${jsk_hrp2_ros_bridge_first_incdir}/jsk_hrp2_ros_bridge)
list(APPEND jsk_hrp2_ros_bridge_INCLUDE_DIRS ${jsk_hrp2_ros_bridge_first_incdir}/jsk_hrp2_ros_bridge/idl)

set(HRP3HANDCONTROLLER_SOURCE ${jsk_hrp2_ros_bridge_SOURCE_PREFIX}/rtc/HRP3HandController)

set(comp_sources handcontrol.cpp ${HRP3HANDCONTROLLER_SOURCE}/HRP3HandController.cpp ${HRP3HANDCONTROLLER_SOURCE}/HRP3HandControllerService_impl.cpp ${SequencePlayerDirectory}/interpolator.cpp)
set(libs hrpIo2_gazebo hrpModel-3.1 hrpCollision-3.1 hrpUtil-3.1 hrpsysBaseStub HRP3HandControllerServiceStub)

if ("$ENV{HRP2NO}" STREQUAL "16")
  set(HRP2NO "-DHRPNO=16")
elseif ("$ENV{HRP2NO}" STREQUAL "17")
  set(HRP2NO "-DHRPNO=17")
else ()
  set(HRP2NO "-DHRPNO=16")
endif()

set(handcontrol_compile_flags "${HRP2NO}")

include_directories(${jsk_hrp2_ros_bridge_INCLUDE_DIRS} ${HRP3HANDCONTROLLER_SOURCE} ${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${SequencePlayerDirectory} ${iobDirectory})
link_directories(${jsk_hrp2_ros_bridge_PREFIX}/lib)

add_definitions(-std=c++14)

message ("compile HRP3HandController")
add_library(HRP3HandController SHARED ${comp_sources})
target_link_libraries(HRP3HandController ${libs})
set_target_properties(HRP3HandController PROPERTIES PREFIX "")
set_target_properties(HRP3HandController PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set_target_properties(HRP3HandController PROPERTIES COMPILE_FLAGS ${handcontrol_compile_flags})

add_executable(HRP3HandControllerComp ${HRP3HANDCONTROLLER_SOURCE}/HRP3HandControllerComp.cpp ${comp_sources})
target_link_libraries(HRP3HandControllerComp ${libs} ${omniorb_LIBRARIES} ${omnidynamic_LIBRARIES} RTC coil dl pthread ${catkin_LIBRARIES})
set_target_properties(HRP3HandControllerComp PROPERTIES COMPILE_FLAGS ${handcontrol_compile_flags})

install(TARGETS HRP3HandController
  LIBRARY DESTINATION lib CONFIGURATIONS Release Debug RELWITHDEBINFO
  )
install(TARGETS HRP3HandController HRP3HandControllerComp
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/lib
  )
