
# setup environment variables for gazebo
PKGPATH=$(rospack find hrpsys_gazebo_tutorials)
export GAZEBO_RESOURCE_PATH=$PKGPATH:$GAZEBO_RESOURCE_PATH
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PKGPATH/robot_models:$PKGPATH/environment_models:$PKGPATH/..
