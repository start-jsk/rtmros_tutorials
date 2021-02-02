#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP
import math

class URATAHrpsysConfigurator(HrpsysConfigurator):
    def __init__(self, robotname=""):
        self.ROBOT_NAME = robotname
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

    def getRTCList (self):
        return self.getRTCListUnstable()

    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        if self.st and self.abc and self.kf and self.ic and self.es and self.el:
            self.setStAbcParameters()
        if self.rmfo:
            self.loadForceMomentOffsetFile()

    def defJointGroups (self):
        if self.ROBOT_NAME == "URATALEG":
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
            self.Groups = [rleg_group, lleg_group]
        elif self.ROBOT_NAME == "YSTLEG":
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT4_2', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT4_2', 'LLEG_JOINT5']]
            self.Groups = [rleg_group, lleg_group]
        elif self.ROBOT_NAME == "CHIDORI":
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
            self.Groups = [rleg_group, lleg_group]
        elif self.ROBOT_NAME == "TQLEG0":
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
            self.Groups = [rleg_group, lleg_group]
        elif self.ROBOT_NAME == "TABLIS":
            rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6']]
            larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6']]
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
            torso_group = ['torso', ['CHEST_JOINT0']]
            self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, torso_group]
        elif self.ROBOT_NAME == "JAXON_BLUE":
            rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6']]
            larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6']]
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
            head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
            torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
            self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group]
        else:
            rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
            larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
            rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
            lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
            head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
            torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
            self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group]

    def setStAbcParameters (self):
        if self.ROBOT_NAME == "STARO":
            self.setStAbcParametersSTARO()
        elif self.ROBOT_NAME == "JAXON":
            self.setStAbcIcParametersJAXON(foot="LEPTRINO")
        elif self.ROBOT_NAME == "JAXON_RED":
            self.setStAbcIcParametersJAXON(foot="KAWADA")
        elif self.ROBOT_NAME == "JAXON_BLUE":
            self.setStAbcIcParametersJAXON_BLUE()
        elif self.ROBOT_NAME == "URATALEG":
            self.setStAbcParametersURATALEG()
        elif self.ROBOT_NAME == "YSTLEG":
            self.setStAbcParametersYSTLEG()
        elif self.ROBOT_NAME == "CHIDORI":
            self.setStAbcParametersCHIDORI()
        elif self.ROBOT_NAME == "TQLEG0":
            self.setStAbcParametersTQLEG0()
        elif self.ROBOT_NAME == "TABLIS":
            self.setStAbcParametersTABLIS()

    def setStAbcParametersSTARO (self):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        abcp.default_zmp_offsets=[[0.00, 0.0, 0.0], [0.00, 0.0, 0.0], [0, 0, 0], [0, 0, 0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
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
        self.st_svc.setParameter(stp)
        # Abc setting
        #gg=self.abc_svc.getGaitGeneratorParam()[1]
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #self.abc_svc.setGaitGeneratorParam(gg)
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.2
        #gg.default_double_support_ratio=0.32
        gg.default_double_support_ratio=0.35
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        gg.swing_trajectory_delay_time_offset=0.35
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        self.abc_svc.setGaitGeneratorParam(gg)


    def setStAbcIcParametersJAXON(self, foot="KAWADA"):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0], [0, 0, 0], [0, 0, 0]];
        abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 0, 0], [0, 0, 0]];
        if self.ROBOT_NAME == "JAXON":
            abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 0, 0], [0, 0, 0]];
        elif self.ROBOT_NAME == "JAXON_RED":
            abcp.default_zmp_offsets=[[0.0, 0.01, 0.0], [0.0, -0.01, 0.0], [0, 0, 0], [0, 0, 0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
        #stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        #stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        stp.emergency_check_mode=OpenHRP.StabilizerService.CP # enable EmergencyStopper for JAXON @ 2015/11/19
        stp.cp_check_margin=[0.05, 0.045, 0, 0.095]
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        #stp.eefm_body_attitude_control_gain=[0, 0.5]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        if self.ROBOT_NAME == "JAXON":
            stp.eefm_rot_damping_gain = [[20*1.6*1.1*1.5*1.2*1.65*1.1, 20*1.6*1.1*1.5*1.2*1.65*1.1, 1e5]]*4
            stp.eefm_pos_damping_gain = [[3500*1.6*6, 3500*1.6*6, 3500*1.6*1.1*1.5*1.2*1.1]]*4
            stp.eefm_swing_rot_damping_gain=[20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5]
            stp.eefm_swing_pos_damping_gain=[3500*1.6*6, 3500*1.6*6, 3500*1.6*1.4]
            stp.eefm_rot_compensation_limit = [math.radians(30), math.radians(30), math.radians(10), math.radians(10)]
            stp.eefm_pos_compensation_limit = [0.06, 0.06, 0.050, 0.050]
        elif self.ROBOT_NAME == "JAXON_RED":
            stp.eefm_rot_damping_gain = [[20*1.6*1.1*1.5, 20*1.6*1.1*1.5, 1e5],
                                         [20*1.6*1.1*1.5, 20*1.6*1.1*1.5, 1e5],
                                         [20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5],
                                         [20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5]]
            stp.eefm_pos_damping_gain = [[3500*1.6*6, 3500*1.6*6, 3500*1.6*1.1*1.5],
                                         [3500*1.6*6, 3500*1.6*6, 3500*1.6*1.1*1.5],
                                         [3500*1.6*6*0.8, 3500*1.6*6*0.8, 3500*1.6*1.1*1.5*0.8],
                                         [3500*1.6*6*0.8, 3500*1.6*6*0.8, 3500*1.6*1.1*1.5*0.8]]
            stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0]
            stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0]
            stp.eefm_rot_compensation_limit = [math.radians(10), math.radians(10), math.radians(10), math.radians(10)]
            stp.eefm_pos_compensation_limit = [0.025, 0.025, 0.050, 0.050]
        stp.eefm_swing_damping_force_thre=[200]*3
        stp.eefm_swing_damping_moment_thre=[15]*3
        stp.eefm_use_swing_damping=True
        stp.eefm_ee_error_cutoff_freq=20.0
        stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*4
        stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*4
        stp.eefm_ee_moment_limit = [[90.0,90.0,1e4], [90.0,90.0,1e4], [1e4]*3, [1e4]*3]
        stp.eefm_rot_time_const = [[1.5/1.1, 1.5/1.1, 1.5/1.1]]*4
        stp.eefm_pos_time_const_support = [[3.0/1.1, 3.0/1.1, 1.5/1.1]]*4
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        if foot == "KAWADA":
            ## KAWADA foot : mechanical param is => inside 0.055, front 0.13, rear 0.1
            stp.eefm_leg_inside_margin=0.05
            stp.eefm_leg_outside_margin=0.05
            stp.eefm_leg_front_margin=0.12
            stp.eefm_leg_rear_margin=0.09
        elif foot == "JSK":
            ## JSK foot : mechanical param is -> inside 0.075, front 0.11, rear 0.11
            stp.eefm_leg_inside_margin=0.07
            stp.eefm_leg_outside_margin=0.07
            stp.eefm_leg_front_margin=0.1
            stp.eefm_leg_rear_margin=0.1
        elif foot == "LEPTRINO":
            stp.eefm_leg_inside_margin=0.05
            stp.eefm_leg_outside_margin=0.05
            stp.eefm_leg_front_margin=0.115
            stp.eefm_leg_rear_margin=0.115
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        stp.eefm_cogvel_cutoff_freq = 4.0
        stp.eefm_k1=[-1.48412,-1.48412]
        stp.eefm_k2=[-0.486727,-0.486727]
        stp.eefm_k3=[-0.198033,-0.198033]
        self.st_svc.setParameter(stp)
        # Abc setting
        #gg=self.abc_svc.getGaitGeneratorParam()[1]
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #self.abc_svc.setGaitGeneratorParam(gg)
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.2
        gg.default_step_height=0.065
        #gg.default_double_support_ratio=0.32
        gg.default_double_support_ratio=0.35
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #gg.swing_trajectory_delay_time_offset=0.35
        #gg.swing_trajectory_delay_time_offset=0.2
        gg.swing_trajectory_delay_time_offset=0.15
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        gg.swing_trajectory_final_distance_weight=3.0
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*117.338;
        gg.heel_pos_offset_x = 1e-3*-116.342;
        gg.toe_zmp_offset_x = 1e-3*117.338;
        gg.heel_zmp_offset_x = 1e-3*-116.342;
        gg.optional_go_pos_finalize_footstep_num=1
        self.abc_svc.setGaitGeneratorParam(gg)
        # Ic setting
        limbs = ['rarm', 'larm']
        for l in limbs:
            icp = self.ic_svc.getImpedanceControllerParam(l)[1]
            icp.D_p = 600
            icp.D_r = 200
            self.ic_svc.setImpedanceControllerParam(l, icp)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setStAbcIcParametersJAXON_BLUE(self, foot="KAWADA"):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        abcp.default_zmp_offsets=[[0.05, 0.0, 0.0], [0.05, 0.0, 0.0], [0, 0, 0], [0, 0, 0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
        #stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        #stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        stp.emergency_check_mode=OpenHRP.StabilizerService.CP
        stp.cp_check_margin=[0.05, 0.045, 0, 0.095]
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        #stp.eefm_body_attitude_control_gain=[0, 0.5]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain = [[25, 25, 1e5], # modification with kojio
                                     [25, 25, 1e5],
                                     [63.36, 63.36, 1e5],
                                     [63.36, 63.36, 1e5]]
        stp.eefm_pos_damping_gain = [[33600.0, 33600.0, 3234.0], # modification with kojio xy=10000?
                                     [33600.0, 33600.0, 3234.0],
                                     [26880.0, 26880.0, 7392.0],
                                     [26880.0, 26880.0, 7392.0]]
        stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0] # same with support leg
        stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0] # same with support leg
        stp.eefm_rot_compensation_limit = [math.radians(10), math.radians(10), math.radians(10), math.radians(10)]
        stp.eefm_pos_compensation_limit = [0.025, 0.025, 0.050, 0.050]
        stp.eefm_swing_damping_force_thre=[200]*3
        stp.eefm_swing_damping_moment_thre=[15]*3
        stp.eefm_use_swing_damping=True
        stp.eefm_ee_error_cutoff_freq=20.0
        # stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*4
        # stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*4
        stp.eefm_ee_moment_limit = [[90.0,90.0,1e4], [90.0,90.0,1e4], [1e4]*3, [1e4]*3]
        stp.eefm_rot_time_const = [[1.5/1.1, 1.5/1.1, 1.5/1.1]]*4
        stp.eefm_pos_time_const_support = [[3.0/1.1, 3.0/1.1, 1.5/1.1]]*4
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        stp.eefm_leg_inside_margin=0.05
        stp.eefm_leg_outside_margin=0.05
        stp.eefm_leg_front_margin=0.16
        stp.eefm_leg_rear_margin=0.06
        rleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_inside_margin])]
        lleg_vertices = [OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, stp.eefm_leg_outside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[stp.eefm_leg_front_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, -1*stp.eefm_leg_inside_margin]),
                         OpenHRP.StabilizerService.TwoDimensionVertex(pos=[-1*stp.eefm_leg_rear_margin, stp.eefm_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
        stp.eefm_cogvel_cutoff_freq = 4.0
        # for only leg
        stp.eefm_k1=[-1.36334,-1.36334]
        stp.eefm_k2=[-0.343983,-0.343983]
        stp.eefm_k3=[-0.161465,-0.161465]
        self.st_svc.setParameter(stp)
        # Abc setting
        #gg=self.abc_svc.getGaitGeneratorParam()[1]
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #self.abc_svc.setGaitGeneratorParam(gg)
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.2
        gg.default_step_height=0.065
        #gg.default_double_support_ratio=0.32
        gg.default_double_support_ratio=0.35
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #gg.swing_trajectory_delay_time_offset=0.35
        #gg.swing_trajectory_delay_time_offset=0.2
        gg.swing_trajectory_delay_time_offset=0.15
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        gg.swing_trajectory_final_distance_weight=3.0
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*117.338;
        gg.heel_pos_offset_x = 1e-3*-116.342;
        gg.toe_zmp_offset_x = 1e-3*117.338;
        gg.heel_zmp_offset_x = 1e-3*-116.342;
        gg.optional_go_pos_finalize_footstep_num=1
        self.abc_svc.setGaitGeneratorParam(gg)
        # Ic setting
        limbs = ['rarm', 'larm']
        for l in limbs:
            icp = self.ic_svc.getImpedanceControllerParam(l)[1]
            icp.D_p = 600
            icp.D_r = 200
            self.ic_svc.setImpedanceControllerParam(l, icp)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setStAbcParametersURATALEG (self):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain=[[20*1.4*5, 20*1.4*5, 1e5]]*2
        stp.eefm_pos_damping_gain=[[3500*10, 3500*10, 3500*1.1*5]]*2
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
        self.st_svc.setParameter(stp)

    def setStAbcParametersYSTLEG (self):
        print "Not implemented yet"

    def setStAbcParametersCHIDORI (self):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0]];
        abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQPCOP
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain=[[20*1.6*1.1*1.5*0.5, 20*1.6*1.1*1.5*0.5, 1e5]]*2
        stp.eefm_pos_damping_gain=[[3500*1.6*3, 3500*1.6*3, 3500*1.6*1.1*1.5*0.5]]*2
        stp.eefm_rot_time_const=[[1.5/1.1, 1.5/1.1, 1.5/1.1]]*2
        stp.eefm_pos_time_const_support=[[1.5/1.1, 1.5/1.1, 1.5/1.1]]*2
        stp.eefm_use_swing_damping=True
        stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0]
        stp.eefm_rot_compensation_limit = [math.radians(30), math.radians(30)]
        stp.eefm_pos_compensation_limit = [0.05, 0.05]
        stp.eefm_ee_error_cutoff_freq=20.0
        stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*2
        stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*2
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        ## KAWADA foot
        #   mechanical param is => inside 0.055, front 0.13, rear 0.1
        # stp.eefm_leg_inside_margin=0.05
        # #stp.eefm_leg_inside_margin=0.04
        # stp.eefm_leg_front_margin=0.12
        # stp.eefm_leg_rear_margin=0.09
        tmp_leg_inside_margin=0.05
        tmp_leg_outside_margin=0.05
        tmp_leg_front_margin=0.12
        tmp_leg_rear_margin=0.09
        # JSK foot
        # # mechanical param is -> inside 0.075, front 0.11, rear 0.11
        # stp.eefm_leg_inside_margin=0.07
        # stp.eefm_leg_front_margin=0.1
        # stp.eefm_leg_rear_margin=0.1
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
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.StabilizerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices])
        # stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq = 4.0
        stp.eefm_k1=[-1.48412,-1.48412]

        # stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq = 4.0
        # calculated by calculate-eefm-st-state-feedback-default-gain-from-robot *chidori*
        stp.eefm_k1=[-1.38444,-1.38444]
        stp.eefm_k2=[-0.368975,-0.368975]
        stp.eefm_k3=[-0.169915,-0.169915]
        self.st_svc.setParameter(stp)
        # Abc setting
        #gg=self.abc_svc.getGaitGeneratorParam()[1]
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #self.abc_svc.setGaitGeneratorParam(gg)
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.2
        #gg.default_double_support_ratio=0.32
        gg.default_double_support_ratio=0.35
        #gg.stride_parameter=[0.1,0.05,10.0]
        #gg.default_step_time=1.0
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        gg.swing_trajectory_final_distance_weight=3.0
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*117.338;
        gg.heel_pos_offset_x = 1e-3*-116.342;
        gg.toe_zmp_offset_x = 1e-3*117.338;
        gg.heel_zmp_offset_x = 1e-3*-116.342;
        self.abc_svc.setGaitGeneratorParam(gg)

    def setStAbcParametersTQLEG0 (self):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        abcp.default_zmp_offsets=[[0.0, 0.045, 0.0], [0.0, -0.045, 0.0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
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
        self.st_svc.setParameter(stp)
        # el setting
        # remove soft-error-limit for torque controlled robot ( ONLY FOR THIS ROBOT !!! )
        self.el_svc.setServoErrorLimit("ALL", 100000)

    def setStAbcParametersTABLIS(self):
        EENUM=4 # end effector num
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]];
        abcp.move_base_gain=0.8
        self.abc_svc.setAutoBalancerParam(abcp)
        # kf setting
        kfp=self.kf_svc.getKalmanFilterParam()[1]
        kfp.R_angle=1000
        self.kf_svc.setKalmanFilterParam(kfp)
        # st setting
        stp=self.st_svc.getParameter()
        stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain=[[200, 200, 1e5]]*EENUM
        stp.eefm_pos_damping_gain=[[30000, 30000, 9000]]*EENUM
        stp.eefm_rot_time_const=[[1.5/1.1, 1.5/1.1, 1.5/1.1]]*EENUM
        stp.eefm_pos_time_const_support=[[1.5/1.1, 1.5/1.1, 1.5/1.1]]*EENUM
        stp.eefm_use_swing_damping=True
        stp.eefm_swing_pos_damping_gain = stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain = stp.eefm_rot_damping_gain[0]
        stp.eefm_rot_compensation_limit = [math.radians(30)]*EENUM
        stp.eefm_pos_compensation_limit = [0.05]*EENUM
        stp.eefm_ee_error_cutoff_freq=20.0
        stp.eefm_swing_rot_spring_gain=[[1.0, 1.0, 1.0]]*EENUM
        stp.eefm_swing_pos_spring_gain=[[1.0, 1.0, 1.0]]*EENUM
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        tmp_leg_inside_margin=0.05
        tmp_leg_outside_margin=0.05
        tmp_leg_front_margin=0.12
        tmp_leg_rear_margin=0.09
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
        # stp.eefm_zmp_delay_time_const=[0.055, 0.055]
        stp.eefm_cogvel_cutoff_freq = 4.0
        # calculated by calculate-eefm-st-state-feedback-default-gain-from-robot *chidori*
        stp.eefm_k1=[-1.38444,-1.38444]
        stp.eefm_k2=[-0.368975,-0.368975]
        stp.eefm_k3=[-0.169915,-0.169915]
        self.st_svc.setParameter(stp)
        # Abc setting
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.2
        gg.default_double_support_ratio=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        gg.swing_trajectory_final_distance_weight=3.0
        gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.toe_pos_offset_x = 1e-3*117.338;
        gg.heel_pos_offset_x = 1e-3*-116.342;
        gg.toe_zmp_offset_x = 1e-3*117.338;
        gg.heel_zmp_offset_x = 1e-3*-116.342;
        self.abc_svc.setGaitGeneratorParam(gg)



    def jaxonResetPose (self):
        ## Different from (send *robot* :reset-pose)
        return [-8.002327e-06,0.000427,-0.244732,0.676564,-0.431836,-0.000427,-8.669072e-06,0.000428,-0.244735,0.676565,-0.431834,-0.000428,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]
    def jaxonBlueResetPose (self):
        ## Different from (send *robot* :reset-pose)
        return [0,0,-0.244732,0.676564,-0.431836,0, 0,0,-0.244735,0.676565,-0.431834,0, 0,0,0, 0,0, 0.698132,-0.349066,-0.087266,-1.39626,0,0,-0.349066, 0.698132,0.349066,0.087266,-1.39626,0,0,-0.349066]

    def staroResetPose(self):
        return [0.174533, 2.0944, 1.5708, 1.5708, -1.0472, 0.0, 0.0, 0.785398, -0.174533, -1.0472, 1.5708, -1.5708, 1.0472, 0.0, 0.0, 0.785398, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0]

    def uratalegResetPose(self):
        return [0.0, 0.0, -0.733038, 1.25664, -0.523599, 0.0, 0.0, 0.0, -0.733038, 1.25664, -0.523599, 0.0]

    def jaxonResetManipPose (self):
        ## Different from (send *robot* :reset-manip-pose)
        return [-7.516871e-06,0.000427,-0.237906,0.673927,-0.436025,-0.000427,-8.257966e-06,0.000428,-0.237909,0.673928,-0.436023,-0.000428,0.0,0.0,0.0,0.0,0.523599,0.0,0.959931,-0.349066,-0.261799,-1.74533,-0.436332,0.0,-0.785398,0.0,0.959931,0.349066,0.261799,-1.74533,0.436332,0.0,-0.785398]

    def jaxonBlueResetManipPose (self):
        ## Different from (send *robot* :reset-manip-pose)
        return [0,0,-0.237906,0.673927,-0.436025,-0.000427, 0,0,-0.237909,0.673928,-0.436023,-0.000428, 0,0,0, 0,0.523599, 0.959931,-0.349066,-0.261799,-1.74533,-0.436332,0,-0.785398, 0.959931,0.349066,0.261799,-1.74533,0.436332,0,-0.785398]

    def staroResetManipPose(self):
        return [0.174533, 2.61799, 1.5708, 1.5708, -1.91986, 0.0, -0.698132, 0.785398, -0.174533, -0.523599, 1.5708, -1.5708, 1.91986, 0.0, 0.698132, 0.785398, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0]

    def jaxonInitPose (self):
        return [0]*len(self.jaxonResetPose())

    def jaxonBlueInitPose (self):
        return [0]*len(self.jaxonBlueResetPose())

    def staroInitPose (self):
        return [0]*len(self.staroResetPose())

    def uratalegInitPose (self):
        return [0]*len(self.uratalegResetPose())

    def jaxonCollisionFreeInitPose (self):
        return [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.261799,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.261799,0.0,0.0,0.0,0.0,0.0]

    def jaxonBlueCollisionFreeInitPose (self):
        return [0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0,0.0,0.0,0.0, 0.0,0.0,0.0, 0.0,0.0, 0.0,-0.261799,0.0,0.0,0.0,0.0,0.0, 0.0,0.261799,0.0,0.0,0.0,0.0,0.0]

    def jaxonCollisionFreeResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.523599,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.523599,0.0,0.0,0.0,0.0,0.0]

    def jaxonBlueCollisionFreeResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0, 0.0,0.0,-0.349066,0.698132,-0.349066,0.0, 0.0,0.0,0.0, 0.0,0.0, 0.0,-0.523599,0.0,0.0,0.0,0.0,0.0, 0.0,0.523599,0.0,0.0,0.0,0.0,0.0]

    def staroResetServoOffPose(self):
        return [1.39626, 1.5708, 0.698132, 1.74533, -0.261799, 0.0, 0.0, -0.785398, -1.39626, -1.5708, 0.698132, -1.39626, 0.261799, 0.0, 0.0, -0.785398, 0.0, 0.0, -0.174533, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.174533, 0.698132, -0.349066, 0.0, 0.0, -0.087266, 0.872665, 0.0]

    def chidoriResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0]

    def chidoriInitPose (self):
        return [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    def tqleg0ResetPose(self):
        return [0.0, 0.0, -0.733038, 1.25664, -0.523599, 0.0, 0.0, 0.0, -0.733038, 1.25664, -0.523599, 0.0]

    def tqleg0InitPose (self):
        return [0]*len(self.tqleg0ResetPose())

    def tablisCollisionFreeInitPose (self):
        return [0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0,
                0, -0.785398163, 0, 0, 0, 0, 0,
                0,  0.785398163, 0, 0, 0, 0, 0]

    def tablisResetPose (self):
        return [0, 0, math.radians(-60), math.radians(90), math.radians(-30), 0,
                0, 0, math.radians(-60), math.radians(90), math.radians(-30), 0,
                0,
                0, math.radians(-30), 0, math.radians(-90), 0, 0, 0,
                0, math.radians(+30), 0, math.radians(-90), 0, 0, 0]

    def tablisLoadTestPose (self):
        return [0, 0, math.radians(-45), math.radians(90), math.radians(-45), 0,
                0, 0, math.radians(-45), math.radians(90), math.radians(-45), 0,
                0,
                0, math.radians(-30), 0, math.radians(-90), 0, 0, 0,
                0, math.radians(+30), 0, math.radians(-90), 0, 0, 0]

    def tablisInitPose (self):
        return [0]*len(self.tablisResetPose())

    # (mapcar #'deg2rad (concatenate cons (send *robot* :reset-landing-pose)))
    def jaxonResetLandingPose (self):
        return [0.004318,0.005074,-0.134838,1.18092,-0.803855,-0.001463,0.004313,0.005079,-0.133569,1.18206,-0.806262,-0.001469,0.003782,-0.034907,0.004684,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]

    def jaxonBlueResetLandingPose (self):
        return self.jaxonBlueResetPose()

    # handmade
    def chidoriResetLandingPose (self):
        return [0.0,0.0,-0.698132,1.39626,-0.698132,0.0,0.0,0.0,-0.698132,1.39626,-0.698132,0.0]

    def setResetPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroResetPose(), 5.0)
        elif self.ROBOT_NAME == "JAXON_BLUE":
            self.seq_svc.setJointAngles(self.jaxonBlueResetPose(), 5.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonResetPose(), 5.0)
        elif self.ROBOT_NAME == "URATALEG":
            self.seq_svc.setJointAngles(self.uratalegResetPose(), 5.0)
        elif self.ROBOT_NAME == "CHIDORI":
            self.seq_svc.setJointAngles(self.chidoriResetPose(), 5.0)
        elif self.ROBOT_NAME == "TQLEG0":
            self.seq_svc.setJointAngles(self.tqleg0ResetPose(), 5.0)
        elif self.ROBOT_NAME == "TABLIS":
            self.seq_svc.setJointAngles(self.tablisResetPose(), 5.0)

    def setLoadTestPose(self):
        if self.ROBOT_NAME == "TABLIS":
            self.seq_svc.setJointAngles(self.tablisLoadTestPose(), 5.0)

    def setResetManipPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroResetManipPose(), 10.0)
        elif self.ROBOT_NAME == "JAXON_BLUE":
            self.seq_svc.setJointAngles(slef.jaxonBlueResetManipPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonResetManipPose(), 10.0)

    def setInitPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroInitPose(), 10.0)
        elif self.ROBOT_NAME == "JAXON_BLUE":
            self.seq_svc.setJointAngles(self.jaxonBlueInitPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonInitPose(), 10.0)
        elif self.ROBOT_NAME == "URATALEG":
            self.seq_svc.setJointAngles(self.uratalegInitPose(), 10.0)
        elif self.ROBOT_NAME == "CHIDORI":
            self.seq_svc.setJointAngles(self.chidoriInitPose(), 10.0)
        elif self.ROBOT_NAME == "TQLEG0":
            self.seq_svc.setJointAngles(self.tqleg0InitPose(), 10.0)
        elif self.ROBOT_NAME == "TABLIS":
            self.seq_svc.setJointAngles(self.tablisInitPose(), 10.0)

    def setCollisionFreeInitPose(self):
        if self.ROBOT_NAME == "JAXON_BLUE":
            self.seq_svc.setJointAngles(self.jaxonBlueCollisionFreeInitPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonCollisionFreeInitPose(), 10.0)
        elif self.ROBOT_NAME == "TABLIS":
            self.seq_svc.setJointAngles(self.tablisCollisionFreeInitPose(), 10.0)

    def setCollisionFreeResetPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroResetServoOffPose(), 10.0)
        elif self.ROBOT_NAME == "JAXON_BLUE":
            self.seq_svc.setJointAngles(self.jaxonBlueCollisionFreeResetPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonCollisionFreeResetPose(), 10.0)

    def setResetLandingPose(self):
        if self.ROBOT_NAME == 0:
            self.seq_svc.setJointAngles(self.jaxonBlueResetLandingPose(), 5.0)
        if self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonResetLandingPose(), 5.0)
        if self.ROBOT_NAME.find("CHIDORI") == 0:
            self.seq_svc.setJointAngles(self.chidoriResetLandingPose(), 5.0)

    def loadForceMomentOffsetFile (self):
        import rospkg
        if self.ROBOT_NAME == "JAXON":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_THK003_CLOSE_JAXON")
        elif self.ROBOT_NAME == "JAXON_RED":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_JAXON_RED")
        else:
            print "No force moment offset file"
