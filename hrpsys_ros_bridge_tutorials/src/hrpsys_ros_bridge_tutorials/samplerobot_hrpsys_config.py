#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

class SampleRobotHrpsysConfigurator(HrpsysConfigurator):
    def getRTCList (self):
        return self.getRTCListUnstable()
    def init (self, robotname="SampleRobot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        print("initialize rtc parameters")
        self.setStAbcParameters()

    def defJointGroups (self):
        rleg_6dof_group = ['rleg', ['RLEG_HIP_R', 'RLEG_HIP_P', 'RLEG_HIP_Y', 'RLEG_KNEE', 'RLEG_ANKLE_P', 'RLEG_ANKLE_R']]
        lleg_6dof_group = ['lleg', ['LLEG_HIP_R', 'LLEG_HIP_P', 'LLEG_HIP_Y', 'LLEG_KNEE', 'LLEG_ANKLE_P', 'LLEG_ANKLE_R']]
        torso_group = ['torso', ['WAIST_P', 'WAIST_R', 'CHEST']]
        head_group = ['head', []]
        rarm_group = ['rarm', ['RARM_SHOULDER_P', 'RARM_SHOULDER_R', 'RARM_SHOULDER_Y', 'RARM_ELBOW', 'RARM_WRIST_Y', 'RARM_WRIST_P', 'RARM_WRIST_R']]
        larm_group = ['larm', ['LARM_SHOULDER_P', 'LARM_SHOULDER_R', 'LARM_SHOULDER_Y', 'LARM_ELBOW', 'LARM_WRIST_Y', 'LARM_WRIST_P', 'LARM_WRIST_R']]
        self.Groups = [rleg_6dof_group, lleg_6dof_group, torso_group, head_group, rarm_group, larm_group]

    def setStAbcParameters (self):
        # ST parameters
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        #   eefm st params
        stp.eefm_leg_inside_margin=71.12*1e-3
        stp.eefm_leg_outside_margin=71.12*1e-3
        stp.eefm_leg_front_margin=182.0*1e-3
        stp.eefm_leg_rear_margin=72.0*1e-3
        stp.eefm_k1=[-1.39899,-1.39899]
        stp.eefm_k2=[-0.386111,-0.386111]
        stp.eefm_k3=[-0.175068,-0.175068]
        stp.eefm_rot_damping_gain=[[20*1.6*10, 20*1.6*10, 1e5]]*4 # Stiff parameter for simulation
        stp.eefm_pos_damping_gain=[[3500*50, 3500*50, 3500*1.0*5]]*4 # Stiff parameter for simulation
        #   tpcc st params
        stp.k_tpcc_p=[0.2, 0.2]
        stp.k_tpcc_x=[4.0, 4.0]
        stp.k_brot_p=[0.0, 0.0]
        stp.k_brot_tc=[0.1, 0.1]
        self.st_svc.setParameter(stp)

    def __init__(self, robotname=""):
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

if __name__ == '__main__':
    hcf = SampleRobotHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
