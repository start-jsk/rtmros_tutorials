#!/usr/bin/env python
# -*- coding: utf-8 -*-

#import rospkg
#import sys
#sys.path.append(rospkg.RosPack().get_path("jsk_hrp2_ros_bridge")+"/scripts")
from jsk_hrp2_gazebo_setup import *
    
def setupHrpsysConfigurator():
    global hcf
    #rtm.nshost = "localhost"
    #rtm.nsport = "2809"
    hcf=JSKHRP2RealHrpsysConfigurator("HRP2JSKNTS")
    hcf.check_argument()

if __name__ == '__main__':
    setupHrpsysConfigurator()
