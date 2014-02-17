#!/usr/bin/env python

PKG = 'hrpsys_ros_bridge_tutorials'
NAME = 'hrpsys_test'

import os
import sys
import time
import unittest
import yaml

import rostest

from subprocess import Popen, PIPE, check_call, call

from hrpsys import rtm
from hrpsys.hrpsys_config import *
import OpenHRP

rtm.nsport = 2809

class TestHrpsys(unittest.TestCase):
    h = None

    def setUp(self):

        global h
        h = HrpsysConfigurator()
        h.init(robotname=sys.argv[1])

    def test_get_joint_angles(self):
        global h
        self.assertEqual(len(h.getJointAngles()), int(sys.argv[2]))

    def test_set_joint_angles(self):
        global h
        self.assertTrue(h.setJointAngles(h.getJointAngles(),1))
        self.assertEqual(h.waitInterpolation(), None)

#unittest.main()
if __name__ == '__main__':
    rostest.run(PKG, NAME, TestHrpsys, sys.argv)


