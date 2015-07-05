#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from jsk_pcl_ros.srv import SetPointCloud2

import os
import commands

def spawn_selected_pointcloud_to_gazebo():
    global create_stl_srv, model_id
    rospy.Subscriber('/selected_pointcloud', PointCloud2, selected_pointcloud_cb)
    model_id = 0
    create_stl_srv = rospy.ServiceProxy('/pointcloud_to_stl/create_stl', SetPointCloud2)

def selected_pointcloud_cb(msg):
    global model_id
    res = create_stl_srv(cloud=msg, name="tmp.stl")
    model_name = "model%d" % model_id
    package_dir = commands.getoutput('rospack find hrpsys_gazebo_tutorials')
    empty_model_dir = package_dir + '/environment_models/empty_model'
    spawn_model_dir = package_dir + '/environment_models/' + model_name
    os.system('cp -r %s %s' % (empty_model_dir, spawn_model_dir))
    os.system('cp /tmp/tmp.stl %s/meshes/root_link_mesh.stl' % spawn_model_dir)
    os.system('sed -i "s/empty_model/%s/g" %s/model.urdf' % (model_name, spawn_model_dir))
    os.system('sed -i "s/empty_model/%s/g" %s/model.config' % (model_name, spawn_model_dir))
    os.system('gzfactory spawn -f %s/model.urdf -z 1 -m %s' % (spawn_model_dir, model_name))
    model_id += 1
    print res.output

if __name__ == '__main__':
    rospy.init_node('spawn_selected_pointcloud_to_gazebo')
    spawn_selected_pointcloud_to_gazebo()
    rospy.spin()
