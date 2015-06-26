#!/usr/bin/env python

import rospy
import math
import jsk_interactive_marker
from geometry_msgs.msg import *
from jsk_interactive_marker.srv import *
from jsk_interactive_marker.msg import *
from jsk_rviz_plugins.msg import TransformableMarkerOperate
from jsk_rviz_plugins.srv import RequestMarkerOperate
import tf

def insert_marker(shape_type=TransformableMarkerOperate.BOX, name='default_name', description='default_description', mesh_resource='', mesh_use_embedded_materials=False):
    try:
        req_marker_operate_srv(TransformableMarkerOperate(type=shape_type, action=TransformableMarkerOperate.INSERT, frame_id="gazebo_world", name=name, description=description, mesh_resource=mesh_resource, mesh_use_embedded_materials=mesh_use_embedded_materials))
    except rospy.ServiceException, e:
        print 'insert_marker service call failed: %s'%e

def insert_kinect_interactive_model():
    global kinect_pose_pub, req_marker_operate_srv, set_pose_srv
    rospy.Subscriber('/transformable_interactive_server/pose', PoseStamped, kinect_marker_pose_cb)
    kinect_pose_pub = rospy.Publisher('/Kinect/SetVelPlugin/PoseCommand', Pose)
    set_control_relative_pose_pub = rospy.Publisher('/transformable_interactive_server/set_control_relative_pose', Pose)
    req_marker_operate_srv = rospy.ServiceProxy('/transformable_interactive_server/request_marker_operate', RequestMarkerOperate)
    set_pose_srv = rospy.ServiceProxy('/transformable_interactive_server/set_control_pose', SetTransformableMarkerPose)
    mesh_resource_name="package://turtlebot_description/meshes/sensors/kinect.dae"
    insert_marker(shape_type=TransformableMarkerOperate.MESH_RESOURCE, name='kinect_marker', description='', mesh_resource=mesh_resource_name, mesh_use_embedded_materials=True)
    rospy.sleep(1)
    set_control_relative_pose_pub.publish(Pose(position=Point(0, -0.031, 0), orientation=Quaternion(*tf.transformations.quaternion_from_euler(0, 0, -math.pi/2))))
    rospy.sleep(1)
    set_pose_srv(pose_stamped=PoseStamped(std_msgs.msg.Header(stamp=rospy.Time.now(), frame_id="camera_link"), Pose(orientation=Quaternion(0, 0, 0, 1))))

def kinect_marker_pose_cb(msg):
    kinect_pose_pub.publish(msg.pose)

if __name__ == '__main__':
    rospy.init_node('insert_kinect_interactive_model')
    rospy.sleep(1)
    insert_kinect_interactive_model()
    rospy.spin()
