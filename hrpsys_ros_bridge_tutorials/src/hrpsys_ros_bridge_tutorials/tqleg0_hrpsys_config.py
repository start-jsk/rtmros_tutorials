#!/usr/bin/env python

from urata_hrpsys_config import *

class TQLEG0HrpsysConfigurator(URATAHrpsysConfigurator):
    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "TQLEG0")

    def resetPose(self):
        return [0.0, 0.0, -0.733038, 1.25664, -0.523599, 0.0, 0.0, 0.0, -0.733038, 1.25664, -0.523599, 0.0]

    def defJointGroups(self):
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        self.Groups = [rleg_group, lleg_group]

    def setDefaultKFParameters(self):
        kfp = self.getKFParameters()
        kfp.R_angle=1000
        self.setKFParameters(kfp)

    def setDefaultABCParameters(self):
        abcp = self.getABCParameters()
        #abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        abcp.default_zmp_offsets=[[0.0, 0.045, 0.0], [0.0, -0.045, 0.0]];
        abcp.move_base_gain=0.8
        self.setABCParameters(abcp)

    def setDefaultSTParameters(self):
        stp = self.getSTParameters()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        #####stp.is_ankle_torque_enable=[True,False]
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain=[[20*1.4*5, 20*1.4*5, 1e5]]*2
        stp.eefm_pos_damping_gain=[[3500*10, 3500*10, 3500*1.1*1.5]]*2
        stp.eefm_rot_time_const=[[1.5, 1.5, 1.5]]*2
        stp.eefm_pos_time_const_support=[[1.5, 1.5, 1.5]]*2
        stp.eefm_wrench_alpha_blending=0.8
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        #   mechanical param is => inside 0.055, front 0.13, rear 0.1
        ## kawada ashi
        # stp.eefm_leg_inside_margin=0.05
        # stp.eefm_leg_front_margin=0.12
        # stp.eefm_leg_rear_margin=0.09
        ## leptrino ashi
        stp.eefm_leg_inside_margin=0.135*0.5
        stp.eefm_leg_front_margin=0.16
        stp.eefm_leg_rear_margin=0.1
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_outside_margin])]
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices])
        stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq = 4.25
        stp.eefm_k1=[-1.38077, -1.38077]
        stp.eefm_k2=[-0.364652, -0.364652]
        stp.eefm_k3=[-0.168538, -0.168538]
        self.setSTParameters(stp)

    def setDefaultServoErrorLimit(self):
        # remove soft-error-limit for torque controlled robot ( ONLY FOR THIS ROBOT !!! )
        self.el_svc.setServoErrorLimit("ALL", 100000)

if __name__ == '__main__':
    hcf = TQLEG0HrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
