#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys

try:
    import hrpsys_tutorials
except:
    import roslib
    roslib.load_manifest('hrpsys_tutorials')

from hrpsys_tutorials import jaxon_client as jaxon
from hrpsys import rtm
import OpenHRP

if __name__ == '__main__':
    hcf = jaxon.JaxonConfigurator()
    # hcf.getRTCList = hcf.getRTCListUnstable

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
