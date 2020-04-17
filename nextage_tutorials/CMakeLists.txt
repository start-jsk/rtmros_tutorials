cmake_minimum_required(VERSION 2.8.3)
project(nextage_tutorials)

find_package(catkin REQUIRED
  nextage_description)

catkin_package()

if(EXISTS ${nextage_description_PREFIX}/share/nextage_description/urdf/NextageOpen.urdf)
  set(nextage_urdf ${nextage_description_PREFIX}/share/nextage_description/urdf/NextageOpen.urdf)
elseif(EXISTS ${nextage_description_SOURCE_PREFIX}/urdf/NextageOpen.urdf)
  set(nextage_urdf ${nextage_description_SOURCE_PREFIX}/urdf/NextageOpen.urdf)
else()
  message(WARNING "Could not found nextage.urdf in ${nextage_description_PREFIX}/share/nextage_description/urdf/NextageOpen.urdf nor ${nextage_description_SOURCE_PREFIX}/urdf/NextageOpen.urdf")
endif()
if (EXISTS ${nextage_urdf})
  message(STATUS "Found nextage.urdf at ${nextage_urdf}")
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/nextage.l
    COMMAND rosrun euscollada collada2eus nextage.dae config/nextage.yaml nextage.l
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS nextage.dae config/nextage.yaml)
  add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/nextage.dae
    COMMAND rosrun collada_urdf urdf_to_collada ${nextage_urdf} nextage.dae
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    DEPENDS ${nextage_urdf})

  # get collada-dom version
  find_package(PkgConfig)
  pkg_check_modules(COLLADA collada-dom>=2.4.4 )
  if ( ${COLLADA_FOUND} )
    add_custom_target(generate_nextage_lisp ALL DEPENDS ${PROJECT_SOURCE_DIR}/nextage.l)
  else()
    pkg_check_modules(COLLADA collada-dom)
    message(WARNING "urdf_to_collada requries collada-dom >= 2.4.4, installed version is ${COLLADA_VERSION}")
  endif()
endif()

#############
## Install ##
#############

install(DIRECTORY config euslisp launch model sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS)

install(FILES nextage.dae nextage.l DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

############
##  Test  ##
############
