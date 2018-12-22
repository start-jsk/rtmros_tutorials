#!/usr/bin/env python

from urata_hrpsys_config import *

class JAXONHrpsysConfigurator(URATAHrpsysConfigurator):
    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "JAXON")

    def resetPose (self):
        ## Different from (send *robot* :reset-pose)
        return [-8.002327e-06,0.000427,-0.244732,0.676564,-0.431836,-0.000427,-8.669072e-06,0.000428,-0.244735,0.676565,-0.431834,-0.000428,0.0,0.0,0.0,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]

    def resetManipPose(self):
        ## Different from (send *robot* :reset-manip-pose)
        return [-7.516871e-06,0.000427,-0.237906,0.673927,-0.436025,-0.000427,-8.257966e-06,0.000428,-0.237909,0.673928,-0.436023,-0.000428,0.0,0.0,0.0,0.0,0.523599,0.0,0.959931,-0.349066,-0.261799,-1.74533,-0.436332,0.0,-0.785398,0.0,0.959931,0.349066,0.261799,-1.74533,0.436332,0.0,-0.785398]

    def resetLandingPose(self):
        # (mapcar #'deg2rad (concatenate cons (send *robot* :reset-landing-pose)))
        return [0.004318,0.005074,-0.134838,1.18092,-0.803855,-0.001463,0.004313,0.005079,-0.133569,1.18206,-0.806262,-0.001469,0.003782,-0.034907,0.004684,0.0,0.0,0.0,0.698132,-0.349066,-0.087266,-1.39626,0.0,0.0,-0.349066,0.0,0.698132,0.349066,0.087266,-1.39626,0.0,0.0,-0.349066]

    def collisionFreeInitPose(self):
        return [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.261799,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.261799,0.0,0.0,0.0,0.0,0.0]

    def collisionFreeResetPose(self):
        return [0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,-0.349066,0.698132,-0.349066,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-0.523599,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.523599,0.0,0.0,0.0,0.0,0.0]

    def defJointGroups(self):
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

    def setDefaultForceMomentOffset(self):
        import rospkg
        self.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_THK003_CLOSE_JAXON")

    def setDefaultESParameters(self):
        esp = self.getESParameters()
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.setESParameters(esp)

    def setDefaultICParameters(self):
        limbs = ['rarm', 'larm']
        for l in limbs:
            icp = self.getICParameters(l)
            icp.D_p = 600
            icp.D_r = 200
            self.setICParameters(l, icp)

    def setDefaultABCParameters(self):
        abcp = self.getABCParameters()
        #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets=[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.move_base_gain=0.8
        self.setABCParameters(abcp)

    def setDefaultGaitGeneraterParameters(self):
        gg = self.getGaitGeneraterParameters()
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
        self.setGaitGeneraterParameters(gg)

    def setDefaultSTParameters(self):
        stp = self.getSTParameters()
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
        stp.eefm_rot_damping_gain = [[20*1.6*1.1*1.5*1.2*1.65*1.1, 20*1.6*1.1*1.5*1.2*1.65*1.1, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*1.6*6, 3500*1.6*6, 3500*1.6*1.1*1.5*1.2*1.1]]*4
        stp.eefm_swing_rot_damping_gain=[20*1.6*1.1*1.5*1.2, 20*1.6*1.1*1.5*1.2, 1e5]
        stp.eefm_swing_pos_damping_gain=[3500*1.6*6, 3500*1.6*6, 3500*1.6*1.4]
        stp.eefm_rot_compensation_limit = [math.radians(30), math.radians(30), math.radians(10), math.radians(10)]
        stp.eefm_pos_compensation_limit = [0.06, 0.06, 0.050, 0.050]
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
        stp.eefm_cogvel_cutoff_freq = 4.0
        stp.eefm_k1=[-1.48412,-1.48412]
        stp.eefm_k2=[-0.486727,-0.486727]
        stp.eefm_k3=[-0.198033,-0.198033]
        self.setSTParameters(stp)
        self.setJAXONFootMarginParam(foot="LEPTRINO")

    def setJAXONFootMarginParam(self, foot="KAWADA"):
        stp=self.getSTParameters()
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
        self.setSTParameters(stp)

if __name__ == '__main__':
    hcf = JAXONHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
