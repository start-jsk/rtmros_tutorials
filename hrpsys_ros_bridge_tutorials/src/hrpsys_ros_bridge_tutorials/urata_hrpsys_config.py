#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

class URATAHrpsysConfigurator(HrpsysConfigurator):
    def __init__(self, robotname=""):
        self.ROBOT_NAME = robotname
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

    def getRTCList (self):
        return self.getRTCListUnstable()

    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        self.setStAbcParameters()

    def defJointGroups (self):
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
            self.setStAbcIcParametersJAXON(foot="KAWADA")
        elif self.ROBOT_NAME == "JAXON_RED":
            self.setStAbcIcParametersJAXON(foot="KAWADA")

    def setStAbcParametersSTARO (self):
        # abc setting
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        abcp.default_zmp_offsets=[[0.00, 0.0, 0.0], [0.00, 0.0, 0.0]];
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
        #stp.eefm_rot_damping_gain=120
        #stp.eefm_rot_damping_gain=70
        #stp.eefm_rot_damping_gain=60
        # old
        #stp.eefm_rot_damping_gain=20*1.6
        # new, servo-off-walking-pose
        #stp.eefm_rot_damping_gain=20*1.75
        # new, reset-manip-psoe
        stp.eefm_rot_damping_gain=20*2.0
        #stp.eefm_rot_damping_gain=100
        #stp.eefm_pos_damping_gain=3500*2.5
        #stp.eefm_pos_damping_gain=7800
        stp.eefm_pos_damping_gain=[3500*1.6*3, 3500*1.6*3*0.07, 3500*1.6]
        #stp.eefm_rot_time_const=1.0
        #stp.eefm_pos_time_const_support=1.0
        stp.eefm_rot_time_const=1.5
        stp.eefm_pos_time_const_support=1.5
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
        #stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        stp.st_algorithm=OpenHRP.StabilizerService.EEFMQP
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        #stp.eefm_body_attitude_control_gain=[0, 0.5]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        if self.ROBOT_NAME == "JAXON":
            stp.eefm_rot_damping_gain=20*1.6*1.1*1.5*1.2
            stp.eefm_pos_damping_gain=[3500*1.6*3, 3500*1.6*3, 3500*1.6*1.1*1.5*1.2]
        elif self.ROBOT_NAME == "JAXON_RED":
            stp.eefm_rot_damping_gain=20*1.6*1.1*1.5
            stp.eefm_pos_damping_gain=[3500*1.6*3, 3500*1.6*3, 3500*1.6*1.1*1.5]
        stp.eefm_rot_time_const=1.5/1.1
        stp.eefm_pos_time_const_support=1.5/1.1
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
            icp.M_p = 0
            icp.D_p = 2000
            icp.K_p = 4000
            icp.moment_gain = [1.0, 1.0, 1.0]
            icp.M_r = 0
            icp.D_r = 150
            icp.K_r = 200
            icp.ik_optional_weight_vector = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
            if self.ROBOT_NAME == "JAXON_RED":
                icp.reference_gain = 0.05
            self.ic_svc.setImpedanceControllerParam(l, icp)

    def jaxonResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]

    def staroResetPose(self):
        return [0.174533, 2.0944, 1.5708, 1.5708, -1.0472, 0.0, 0.0, 0.785398, -0.174533, -1.0472, 1.5708, -1.5708, 1.0472, 0.0, 0.0, 0.785398, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0]

    def jaxonResetManipPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.959931,-0.349066,-0.261799,-1.74533,-0.436332,0.0,-0.785398,0.0,0.959931,0.349066,0.261799,-1.74533,0.436332,0.0,-0.785398]

    def staroResetManipPose(self):
        return [0.174533, 2.61799, 1.5708, 1.5708, -1.91986, 0.0, -0.698132, 0.785398, -0.174533, -0.523599, 1.5708, -1.5708, 1.91986, 0.0, 0.698132, 0.785398, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0, 0.0, 0.0, 0.0, 0.0]

    def jaxonInitPose (self):
        return [0]*len(self.jaxonResetPose())

    def staroInitPose (self):
        return [0]*len(self.staroResetPose())

    def jaxonCollisionFreeInitPose (self):
        return [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.261799,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.261799,0.0,0.0,0.0,0.0,0.0]

    def jaxonCollisionFreeResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.523599,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.523599,0.0,0.0,0.0,0.0,0.0]

    def staroResetServoOffPose(self):
        return [1.39626, 1.5708, 0.698132, 1.74533, -0.261799, 0.0, 0.0, -0.785398, -1.39626, -1.5708, 0.698132, -1.39626, 0.261799, 0.0, 0.0, -0.785398, 0.0, 0.0, -0.174533, 0.698132, -0.349066, 0.0, 0.0, 0.0, -0.174533, 0.698132, -0.349066, 0.0, 0.0, -0.087266, 0.872665, 0.0]

    def setResetPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroResetPose(), 5.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonResetPose(), 5.0)

    def setResetManipPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroResetManipPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonResetManipPose(), 10.0)

    def setInitPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroInitPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonInitPose(), 10.0)

    def setCollisionFreeInitPose(self):
        if self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonCollisionFreeInitPose(), 10.0)

    def setCollisionFreeResetPose(self):
        if self.ROBOT_NAME == "STARO":
            self.seq_svc.setJointAngles(self.staroResetServoOffPose(), 10.0)
        elif self.ROBOT_NAME.find("JAXON") == 0:
            self.seq_svc.setJointAngles(self.jaxonCollisionFreeResetPose(), 10.0)
