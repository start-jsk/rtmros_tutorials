cmake_minimum_required(VERSION 2.8.3)
project(openhrp3_tutorials)

find_package(catkin REQUIRED COMPONENTS)
catkin_package()

add_custom_command(OUTPUT ${PROJECT_SOURCE_DIR}/test/.gen_testcode
  COMMAND make -C ${PROJECT_SOURCE_DIR}/test/
  VERBATIM)
add_custom_target(gen_testcode DEPENDS test/barrett-wam.zae ${PROJECT_SOURCE_DIR}/test/.gen_testcode)
add_dependencies(tests gen_testcode)

#download_test_data(https://openrave.svn.sourceforge.net/svnroot/openrave/data/robots/barrett-wam.zae test/barrett-wam.zae 7d59cad7186b3fb516c3dd75303c52f2)
#add_rostest(test/test-modelloader.launch)
