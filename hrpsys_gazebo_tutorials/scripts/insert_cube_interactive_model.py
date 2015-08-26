#!/usr/bin/env python

import rospy
import math
import jsk_interactive_marker
from geometry_msgs.msg import *
from jsk_interactive_marker.srv import *
from jsk_interactive_marker.msg import *
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *
from jsk_rviz_plugins.msg import TransformableMarkerOperate
from jsk_rviz_plugins.srv import RequestMarkerOperate
from gazebo_msgs.srv import SpawnModel
import tf
import rospkg

def disable_gravity():
    rospy.wait_for_service('/gazebo/set_physics_properties')
    set_physics_properties_srv = rospy.ServiceProxy('/gazebo/set_physics_properties', SetPhysicsProperties)
    get_physics_properties_srv = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
    current_physics_properties = get_physics_properties_srv()
    set_physics_properties_srv(time_step=0.001,
                               max_update_rate=1000,
                               gravity=Vector3(x=0, y=0, z=0),
                               ode_config=current_physics_properties.ode_config)

def spawn_gazebo_cube_model():
    try:
        spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        urdf_file_path = rospkg.RosPack().get_path('hrpsys_gazebo_tutorials')+'/environment_models/Cube/Cube.urdf'
        urdf_file_data = open(urdf_file_path).read()
        spawn_model_srv(model_name='Cube', model_xml=urdf_file_data)
    except rospy.ServiceException, e:
        print 'spawn_model service call failed: %s'%e

def insert_marker(shape_type=TransformableMarkerOperate.BOX, name='default_name', description='default_description', mesh_resource='', mesh_use_embedded_materials=False):
    try:
        req_marker_operate_srv(TransformableMarkerOperate(type=shape_type, action=TransformableMarkerOperate.INSERT, frame_id="gazebo_world", name=name, description=description, mesh_resource=mesh_resource, mesh_use_embedded_materials=mesh_use_embedded_materials))
    except rospy.ServiceException, e:
        print 'insert_marker service call failed: %s'%e

def insert_cube_interactive_model():
    global cube_pose_pub, req_marker_operate_srv, set_pose_srv
    rospy.Subscriber('/transformable_interactive_server/pose_with_name', PoseStampedWithName, cube_marker_pose_cb)
    cube_pose_pub = rospy.Publisher('/Cube/SetVelPlugin/PoseCommand', Pose)
    req_marker_operate_srv = rospy.ServiceProxy('/transformable_interactive_server/request_marker_operate', RequestMarkerOperate)
    mesh_resource_name="package://hrpsys_gazebo_tutorials/environment_models/Cube/meshes/nil_link_mesh.dae"
    insert_marker(shape_type=TransformableMarkerOperate.MESH_RESOURCE, name='cube_marker', description='', mesh_resource=mesh_resource_name, mesh_use_embedded_materials=True)

def cube_marker_pose_cb(msg):
    if msg.name == 'cube_marker':
        cube_pose_pub.publish(msg.pose.pose)

if __name__ == '__main__':
    rospy.init_node('insert_cube_interactive_model')
    disable_gravity()
    rospy.sleep(1)
    spawn_gazebo_cube_model()
    rospy.sleep(1)
    insert_cube_interactive_model()
    rospy.spin()
