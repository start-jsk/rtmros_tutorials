#!/usr/bin/env python

from urata_hrpsys_config import *

class STAROHrpsysConfigurator(URATAHrpsysConfigurator):
    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "STARO")

    def resetPose(self):
        return [0.174533, 2.0944, 1.5708, 1.5708, -1.0472, 0.0, 0.0, 0.785398, -0.174533, -1.0472, 1.5708, -1.5708, 1.0472, 0.0, 0.0, 0.785398, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0]

    def resetManipPose(self):
        return [0.174533, 2.61799, 1.5708, 1.5708, -1.91986, 0.0, -0.698132, 0.785398, -0.174533, -0.523599, 1.5708, -1.5708, 1.91986, 0.0, 0.698132, 0.785398, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0]

    def resetServoOffPose(self):
        return [1.39626, 1.5708, 0.698132, 1.74533, -0.261799, 0.0, 0.0, -0.785398, -1.39626, -1.5708, 0.698132, -1.39626, 0.261799, 0.0, 0.0, -0.785398, 0.0, 0.0, -0.174533, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.174533, 0.698132, -0.349066, 0.0, 0.0, -0.087266, 0.872665, 0.0]

    def defJointGroups(self):
        # Is it correct?
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
        self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group]

    def setDefaultKFParameters(self):
        kfp = self.getKFParameters()
        kfp.R_angle=1000
        self.setKFParameters(kfp)

    def setDefaultABCParameters(self):
        abcp = self.getABCParameters()
        abcp.default_zmp_offsets=[[0.00, 0.0, 0.0], [0.00, 0.0, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.move_base_gain=0.8
        self.setABCParameters(abcp)

    def setDefaultGaitGeneraterParameters(self):
        gg = self.getGaitGeneraterParameters()
        gg.default_step_time=1.2
        #gg.default_double_support_ratio=0.32
        gg.default_double_support_ratio=0.35
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        gg.swing_trajectory_delay_time_offset=0.35
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        self.setGaitGeneraterParameters(gg)

    def setDefaultSTParameters(self):
        stp = self.getSTParameters()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        #stp.emergency_check_mode=OpenHRP.StabilizerService.CP
        #stp.cp_check_margin=80*1e-3
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        #stp.eefm_rot_damping_gain=120
        #stp.eefm_rot_damping_gain=70
        #stp.eefm_rot_damping_gain=60
        # old
        #stp.eefm_rot_damping_gain=20*1.6
        # new, servo-off-walking-pose
        #stp.eefm_rot_damping_gain=20*1.75
        # new, reset-manip-psoe
        stp.eefm_rot_damping_gain = [[20*2.0, 20*2.0, 1e5]]*4
        #stp.eefm_rot_damping_gain=100
        #stp.eefm_pos_damping_gain=3500*2.5
        #stp.eefm_pos_damping_gain=7800
        stp.eefm_pos_damping_gain = [[3500*1.6*3, 3500*1.6*3*0.07, 3500*1.6]]*4
        #stp.eefm_rot_time_const=1.0
        #stp.eefm_pos_time_const_support=1.0
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_wrench_alpha_blending=0.7
        #stp.eefm_pos_time_const_swing=0.03
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        #   mechanical param is => inside 0.055, front 0.13, rear 0.1
        stp.eefm_leg_inside_margin=0.05
        stp.eefm_leg_outside_margin=0.05
        #stp.eefm_leg_inside_margin=0.04
        stp.eefm_leg_front_margin=0.12
        stp.eefm_leg_rear_margin=0.09
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
        #stp.eefm_cogvel_cutoff_freq = 3.18
        #stp.eefm_cogvel_cutoff_freq = 6.18 # servooff+walk
        #stp.eefm_cogvel_cutoff_freq = 5.5
        #stp.eefm_cogvel_cutoff_freq = 4.5
        #stp.eefm_cogvel_cutoff_freq = 3.5 # reset-pose
        #stp.eefm_cogvel_cutoff_freq = 5.0
        # old
        #stp.eefm_cogvel_cutoff_freq = 4.25
        # new
        stp.eefm_cogvel_cutoff_freq = 4.0
        # old
        # stp.eefm_k1=[-1.46719, -1.46719]
        # stp.eefm_k2=[-0.466538, -0.466538]
        # stp.eefm_k3=[-0.194206, -0.194206]
        # new, servo-off-walking-pose
        # stp.eefm_k1=[-1.45044,-1.45044]
        # stp.eefm_k2=[-0.44668,-0.44668]
        # stp.eefm_k3=[-0.190114,-0.190114]
        # new, reset-manip-pose
        stp.eefm_k1=[-1.48696,-1.48696]
        stp.eefm_k2=[-0.490137,-0.490137]
        stp.eefm_k3=[-0.198649,-0.198649]
        # new, reset-manip-pose
        self.setSTParameters(stp)

if __name__ == '__main__':
    hcf = STAROHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
