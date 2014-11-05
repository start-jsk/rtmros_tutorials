#!/usr/bin/env python

import math
import rospy
import tf
import numpy
from tf.transformations import *
from urdf_parser_py.urdf import URDF

def publishEndEffectorOne(eef_name, eef_info, stamp):
    br = tf.TransformBroadcaster()
    br.sendTransform(eef_info['translate'], quaternion_about_axis(math.radians(eef_info['rotate'][3]), eef_info['rotate'][:3]),
                     stamp, eef_name, eef_info['parent'])

def publishEndEffectorAll():
    for limb_name, eef_info in g_eef_info_list.items():
        publishEndEffectorOne(limb_name+"_end_coords", eef_info, rospy.Time.now())

if __name__ == "__main__":
    rospy.init_node('publish_end_effector_tf')
    tf_listener = tf.TransformListener()
    r = rospy.Rate(10)
    g_eef_info_list = {}
    limb_list = ['larm', 'rarm', 'lleg', 'rleg', 'head']
    robot_model = URDF.from_xml_string(rospy.get_param("/robot_description"))
    for limb in limb_list:
        # look for parent link of end effector
        parent_link_name = ''
        for link_name in robot_model.link_map:
            if link_name.startswith(limb.upper()) or link_name.startswith(limb.lower()):
                parent_link_name = link_name
                break
        parent_link_name_next = parent_link_name
        while True:
            parent_link_name = parent_link_name_next
            if robot_model.child_map.has_key(parent_link_name):
                parent_link_name_next = robot_model.child_map[parent_link_name][0][1]
                if not (parent_link_name_next.startswith(limb.upper()) or parent_link_name_next.startswith(limb.lower())):
                    break
            else:
                break
        # get ros param of end effector
        param_name = '~'+limb+'-end-coords'
        if rospy.has_param(param_name):
            g_eef_info_list[limb] = rospy.get_param(param_name)
            if not g_eef_info_list[limb].has_key('translate'):
                g_eef_info_list[limb]['translate'] = [0, 0, 0, 0]
            if not g_eef_info_list[limb].has_key('rotate'):
                g_eef_info_list[limb]['rotate'] = [0, 0, 0, 0]
            if not g_eef_info_list[limb].has_key('parent'):
                g_eef_info_list[limb]['parent'] = parent_link_name
    # print for debug
    print 'eef_infos param list:'
    for limb, eef_info in g_eef_info_list.items():
        print '%s: %s' % (limb, eef_info)

    while not rospy.is_shutdown():
        publishEndEffectorAll()
        r.sleep()

