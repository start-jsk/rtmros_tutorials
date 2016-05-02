#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

class JSKHRP4HrpsysConfigurator(HrpsysConfigurator):
    ROBOT_NAME = None

    def getRTCList (self):
        return self.getRTCListUnstable()
    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        print "initialize rtc parameters"
        self.setStAbcParameters()

    def defJointGroups (self):
        rleg_group = ['rleg', ['R_HIP_Y', 'R_HIP_R', 'R_HIP_P', 'R_KNEE_P', 'R_ANKLE_P', 'R_ANKLE_R']]
        lleg_group = ['lleg', ['L_HIP_Y', 'L_HIP_R', 'L_HIP_P', 'L_KNEE_P', 'L_ANKLE_P', 'L_ANKLE_R']]
        torso_group = ['torso', ['CHEST_P', 'CHEST_Y']]
        head_group = ['head', ['NECK_Y', 'NECK_P']]
        rarm_group = ['rarm', ['R_SHOULDER_P', 'R_SHOULDER_R', 'R_SHOULDER_Y', 'R_ELBOW_P', 'R_WRIST_Y', 'R_WRIST_P', 'R_WRIST_R', 'R_HAND_J0', 'R_HAND_J1']]
        larm_group = ['larm', ['L_SHOULDER_P', 'L_SHOULDER_R', 'L_SHOULDER_Y', 'L_ELBOW_P', 'L_WRIST_Y', 'L_WRIST_P', 'L_WRIST_R', 'L_HAND_J0', 'L_HAND_J1']]
        self.Groups = [rleg_group, lleg_group, torso_group, head_group, rarm_group, larm_group]

    def hrp4ResetPose (self):
        if self.ROBOT_NAME.find("HRP4R") != -1:
            return [0.0,-1.0,-20.0,40.0,-20.0,1.0,0.0,1.0,-20.0,40.0,-20.0,-1.0,8.0,0.0,0.0,0.0,-3.0,-10.0,0.0,-30.0,0.0,0.0,0.0,1.0,0.0,-3.0,10.0,0.0,-30.0,0.0,0.0,0.0,-1.0,0.0]
        else:
            return [0.0,-1.0,-20.0,40.0,-20.0,1.0,0.0,1.0,-20.0,40.0,-20.0,-1.0,8.0,0.0,0.0,0.0,-3.0,-10.0,0.0,-30.0,0.0,0.0,0.0,1.0,0.0,-3.0,10.0,0.0,-30.0,0.0,0.0,0.0,-1.0,0.0]
    def hrp4ResetManipPose (self):
        if self.ROBOT_NAME.find("HRP4R") != -1:
            return [0.0,-1.0,-20.0,40.0,-20.0,1.0,0.0,1.0,-20.0,40.0,-20.0,-1.0,8.0,0.0,0.0,15.0,50.0,-30.0,-10.0,-120.0,-25.0,-20.0,-5.0,1.0,0.0,50.0,30.0,10.0,-120.0,25.0,-20.0,5.0,-1.0,0.0]
        else:
            return [0.0,-1.0,-20.0,40.0,-20.0,1.0,0.0,1.0,-20.0,40.0,-20.0,-1.0,8.0,0.0,0.0,15.0,50.0,-30.0,-10.0,-120.0,-25.0,-20.0,-5.0,1.0,0.0,50.0,30.0,10.0,-120.0,25.0,-20.0,5.0,-1.0,0.0]

    def hrp4InitPose (self):
        if self.ROBOT_NAME.find("HRP4R") != -1:
            return [0]*len(self.hrp4ResetPose())

    def setStAbcParameters (self):
        if self.ROBOT_NAME == "HRP4R":
            self.setStAbcParametershrp4001c()

    def setStAbcParametershrp4001c (self):
        # ABC parameters
        abcp = self.abc_svc.getAutoBalancerParam()[1]
        abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp = self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        # stp.st_algorithm=OpenHRP.StabilizerService.TPCC
        # stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        # eefm st params
        #stp.eefm_body_attitude_control_gain=[5, 5]
        stp.eefm_body_attitude_control_gain=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        #stp.eefm_rot_damping_gain=20*3
        #stp.eefm_pos_damping_gain=3500*3
        #stp.eefm_rot_time_const=1.0
        #stp.eefm_pos_time_const_support=1.0
        #stp.eefm_rot_damping_gain=20*2.5
        #stp.eefm_pos_damping_gain=3500*2.5
        #stp.eefm_rot_damping_gain=20*1.4
        #stp.eefm_pos_damping_gain=3500*1.0
        # EEFM parameters for 4 limbs

        # stp.eefm_rot_damping_gain = [[20*1.6, 20*1.6, 1e5]]*4
        # stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0]]*4
        stp.eefm_rot_damping_gain = [[20*1.6*1.2, 20*1.6*1.2, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0*1.2]]*4

        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_wrench_alpha_blending = 0.6
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        #stp.eefm_cogvel_cutoff_freq=3.181
        #stp.eefm_cogvel_cutoff_freq=4.0
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.065
        #stp.eefm_leg_front_margin=0.140
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.055
        tmp_leg_outside_margin=0.070
        # tmp_leg_outside_margin=0.0
        tmp_leg_front_margin=0.125
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        #   tpcc st params
        stp.k_tpcc_p=[2.0, 2.0]
        stp.k_tpcc_x=[5.0, 5.0]
        stp.k_brot_p=[0.0, 0.0]
        stp.k_brot_tc=[0.1, 0.1]
        #   cog height = 800[mm], alpha = -13.0, beta = -4.0, time_const = 0.04[s]
        #stp.eefm_k1=[-1.41413,-1.41413]
        #stp.eefm_k2=[-0.403901,-0.403901]
        #stp.eefm_k3=[-0.179953,-0.179953]
        stp.eefm_k1=[-1.272861, -1.272861]
        stp.eefm_k2=[-0.36367379999999999, -0.36367379999999999]
        stp.eefm_k3=[-0.16200000000000001, -0.16200000000000001]
        # for estop
        stp.emergency_check_mode=OpenHRP.StabilizerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        self.st_svc.setParameter(stp)
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.1
        gg.default_double_support_ratio=0.32
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        gg.swing_trajectory_final_distance_weight=3.0
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*142.869;
        gg.heel_pos_offset_x = 1e-3*-105.784;
        gg.toe_zmp_offset_x = 1e-3*79.411;
        gg.heel_zmp_offset_x = 1e-3*-105.784;
        gg.use_toe_joint = True
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        # esp=self.es_svc.getEmergencyStopperParam()[1]
        # esp.default_recover_time=10.0 # [s]
        # esp.default_retrieve_time=1.0 # [s]
        # self.es_svc.setEmergencyStopperParam(esp)

    def setResetPose(self):
        # self.seq_svc.setJointAngles(self.hrp4ResetPose(), 5.0)
        self.setJointAngles(self.hrp4ResetPose(), 5.0)

    def setResetManipPose(self):
        self.seq_svc.setJointAngles(self.hrp4ResetManipPose(), 5.0)
        self.setJointAngles(self.hrp4ResetManipPose(), 5.0)

    def setInitPose(self):
        self.seq_svc.setJointAngles(self.hrp4InitPose(), 5.0)
        self.setJointAngles(self.hrp4InitPose(), 5.0)

    def __init__(self, robotname=""):
        self.ROBOT_NAME = robotname
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()
