#!/usr/bin/env python

from urata_hrpsys_config import *

class YSTLEGHrpsysConfigurator(URATAHrpsysConfigurator):
    """
    Subclass for YSTLEG configuration.
    Please inherit this class to specify environmnet-dependent class.
    """

    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "YSTLEG")

    def defJointGroups(self):
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT4_2', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT4_2', 'LLEG_JOINT5']]
        self.Groups = [rleg_group, lleg_group]

if __name__ == '__main__':
    hcf = YSTLEGHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
