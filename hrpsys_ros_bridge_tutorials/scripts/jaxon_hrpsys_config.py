#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

class JAXONHrpsysConfigurator(HrpsysConfigurator):
    def getRTCList (self):
        return self.getRTCListUnstable()
    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        self.setStAbcParam()

    def defJointGroups (self):
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        rleg_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1', 'CHEST_JOINT2']]
        self.Groups = [rarm_group, larm_group, rleg_group, lleg_group, head_group, torso_group]

    def setStAbcParam(self):
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
        stp.st_algorithm=OpenHRP.StabilizerService.EEFM
        stp.k_brot_p=[0, 0]
        stp.k_brot_tc=[1000, 1000]
        stp.eefm_body_attitude_control_gain=[0.5, 0.5]
        stp.eefm_body_attitude_control_time_const=[1000, 1000]
        stp.eefm_rot_damping_gain=20*1.6*1.1*1.5
        stp.eefm_pos_damping_gain=[3500*1.6*3, 3500*1.6*3, 3500*1.6*1.1*1.5]
        stp.eefm_rot_time_const=1.5/1.1
        stp.eefm_pos_time_const_support=1.5/1.1
        stp.eefm_wrench_alpha_blending=0.7
        stp.eefm_pos_time_const_swing=0.06
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        # foot margin param
        ## KAWADA foot
        #   mechanical param is => inside 0.055, front 0.13, rear 0.1
        stp.eefm_leg_inside_margin=0.05
        #stp.eefm_leg_inside_margin=0.04
        stp.eefm_leg_front_margin=0.12
        stp.eefm_leg_rear_margin=0.09
        ## JSK foot
        # # mechanical param is -> inside 0.075, front 0.11, rear 0.11
        # stp.eefm_leg_inside_margin=0.07
        # stp.eefm_leg_front_margin=0.1
        # stp.eefm_leg_rear_margin=0.1
        # stp.eefm_zmp_delay_time_const=[0.055, 0.055]
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

    def jaxonResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]

    def jaxonResetManipPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.959931,-0.349066,-0.261799,-1.74533,-0.436332,0.0,-0.785398,0.0,0.959931,0.349066,0.261799,-1.74533,0.436332,0.0,-0.785398]

    def jaxonInitPose (self):
        return [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0]

    def jaxonCollisionFreeInitPose (self):
        return [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.261799,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.261799,0.0,0.0,0.0,0.0,0.0]

    def jaxonCollisionFreeResetPose (self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.523599,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.523599,0.0,0.0,0.0,0.0,0.0]

    def setResetPose(self):
        self.seq_svc.setJointAngles(self.jaxonResetPose(), 10.0)

    def setResetManipPose(self):
        self.seq_svc.setJointAngles(self.jaxonResetManipPose(), 10.0)

    def setInitPose(self):
        self.seq_svc.setJointAngles(self.jaxonInitPose(), 10.0)

    def setCollisionFreeInitPose(self):
        self.seq_svc.setJointAngles(self.jaxonCollisionFreeInitPose(), 10.0)

    def __init__(self):
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

if __name__ == '__main__':
    hcf=JAXONHrpsysConfigurator()
    if len(sys.argv) != 2:
        hcf.init(sys.argv[1], sys.argv[2])
