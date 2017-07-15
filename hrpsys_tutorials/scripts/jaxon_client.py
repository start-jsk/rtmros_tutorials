#!/usr/bin/env python

import sys

from hrpsys import rtm
from hrpsys.hrpsys_config import *
import OpenHRP

#
class JaxonConfigurator(HrpsysConfigurator):

    def goPos(self, x, y, th):
        '''!@brief Walk to the goal position and orientation. Returns without waiting for whole steps to be executed.
        @param i_x[m], i_y[m], and i_th[deg] are goal x-y-position and z-orientation from the current mid-coords of right foot and left foot.
        @return true if set successfully, false otherwise'''
        self.abc_svc.goPos(x, y, th)

    def goVelocity(self, vx, vy, vth):
        '''!@brief Walk at the desired velocity. If the robot is stopping, the robot starts stepping. Returns without waiting for whole steps to be executed.
        @param i_vx[m/s], i_vy[m/s], and i_vth[deg/s] are velocity in the current mid-coords of right foot and left foot.
        @return true if set successfully, false otherwise'''
        self.abc_svc.goVelocity(vx, vy, vth)

    def goStop(self):
        '''!@brief Stop stepping.
        @param
        @return true if set successfully, false otherwise'''
        self.abc_svc.goStop()

if __name__ == '__main__':
    hcf = JaxonConfigurator()
    hcf.getRTCList = hcf.getRTCListUnstable

    # initialize if we have arguments
    if len(sys.argv) > 2:
        hcf.waitForRTCManagerAndRoboHardware(robotname=sys.argv[1])
        hcf.init(sys.argv[1], sys.argv[2])
    else:
        hcf.findComps()

    # if auto balancer is not started yet
    if hcf.abc_svc.getAutoBalancerParam()[1].controller_mode != OpenHRP.AutoBalancerService.MODE_ABC:
        hcf.setJointAngles(
            [0.0, 0.0, -20.0, 40.0, -20.0, 0.0,
             0.0, 0.0, -20.0, 40.0, -20.0, 0.0,
             0.0, 0.0, 0.0,
             0.0, 0.0,
             0.0, +40.0, -20.0, -5.0, -80.0, 0.0, 0.0, -20.0,
             0.0, +40.0, +20.0, +5.0, -80.0, 0.0, 0.0, -20.0,
             0,0,0,0], 2)
        hcf.waitInterpolation()
        hcf.startAutoBalancer()
    
