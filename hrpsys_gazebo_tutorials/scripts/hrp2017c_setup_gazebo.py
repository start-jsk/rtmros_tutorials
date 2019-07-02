#!/usr/bin/env python
# -*- coding: utf-8 -*-

from jsk_hrp2_gazebo_setup import *

def setupHrpsysConfigurator():
    global hcf
    rtm.nshost = "localhost"
    hcf=JSKHRP2RealHrpsysConfigurator("HRP2JSKNTS")
    hcf.check_argument()

if __name__ == '__main__':
    setupHrpsysConfigurator()
