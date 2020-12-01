#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP

pkg = 'jsk_hrp2_ros_bridge'
import imp
try:
    imp.find_module(pkg)
except:
    print "No such package ", pkg

try:
    from jsk_hrp2_ros_bridge.HRP3HandControllerService_idl import *
except:
    print """No HRP3hand module is availabe on this machine,
it may cause some error"""

class JSKHRP2HrpsysConfigurator(HrpsysConfigurator):
    ROBOT_NAME = None

    hc = None
    hc_svc = None

    def connectComps(self):
        HrpsysConfigurator.connectComps(self)
        if self.rh.port("servoState") != None:
            if self.hc:
                connectPorts(self.rh.port("servoState"), self.hc.port("servoStateIn"))

    def getRTCList(self):
        rtclist = [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            #['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            #['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            # ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            #['tc', "TorqueController"],
            ['te', "ThermoEstimator"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['tl', "ThermoLimiter"],
            ['bp', "Beeper"],
            ['log', "DataLogger"],
            ]
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            rtclist.append(['hc', "HRP3HandController"])
        if self.ROBOT_NAME.find("HRP2JSKNT") == -1: # if HRP2W, HRP2G and HRP2JSK
            tmpidx=rtclist.index(['ic', "ImpedanceController"])
            rtclist[0:tmpidx+1]+rtclist[tmpidx+1:]
            rtclist=rtclist[0:tmpidx+1]+[['gc', "GraspController"]]+rtclist[tmpidx+1:]
        print >>sys.stderr, "RTC ", rtclist, "[",self.ROBOT_NAME.find("HRP2JSKNT"),"][",self.ROBOT_NAME,"]"
        return rtclist

    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        print "initialize rtc parameters"
        self.setStAbcParameters()
        self.loadForceMomentOffsetFile()

    def defJointGroups (self):
        rleg_6dof_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5']]
        lleg_6dof_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5']]
        rleg_7dof_group = ['rleg', ['RLEG_JOINT0', 'RLEG_JOINT1', 'RLEG_JOINT2', 'RLEG_JOINT3', 'RLEG_JOINT4', 'RLEG_JOINT5', 'RLEG_JOINT6']]
        lleg_7dof_group = ['lleg', ['LLEG_JOINT0', 'LLEG_JOINT1', 'LLEG_JOINT2', 'LLEG_JOINT3', 'LLEG_JOINT4', 'LLEG_JOINT5', 'LLEG_JOINT6']]
        torso_group = ['torso', ['CHEST_JOINT0', 'CHEST_JOINT1']]
        head_group = ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']]
        rarm_group = ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']]
        larm_group = ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]
        if self.ROBOT_NAME == "HRP2JSKNT" or self.ROBOT_NAME == "HRP2JSKNTS":
            self.Groups = [rleg_7dof_group, lleg_7dof_group, torso_group, head_group, rarm_group, larm_group]
        elif self.ROBOT_NAME == "HRP2JSK":
            self.Groups = [rleg_6dof_group, lleg_6dof_group, torso_group, head_group, rarm_group, larm_group]
        else: # HRP2W, HRP2G
            self.Groups = [torso_group, head_group, rarm_group, larm_group]

    def hrp2ResetPose (self):
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.174533, -0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, 0.261799, 0.174533, 0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, -0.261799]
        elif self.ROBOT_NAME.find("HRP2JSK") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.174533, -0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, 0.261799, 0.174533, 0.174533, 0.0, -0.436332, 0.0, 0.0, -0.174533, -0.261799]
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.174533, -0.174533, 0.0, -1.5708, 0.0, 0.0, -0.174533, 0.261799, 0.174533, 0.174533, 0.0, -1.5708, 0.0, 0.0, -0.174533, -0.261799]

    def hrp2ResetManipPose (self):
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.0, 0.698132, 0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472, 0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472]
        elif self.ROBOT_NAME.find("HRP2JSK") != -1:
            return [0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, 0.0, 0.698132, 0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472, 0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472]
        else:
            return [0.0, 0.0, 0.0, 0.698132, 0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472, 0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472]

    def hrp2InitPose (self):
        if self.ROBOT_NAME.find("HRP2JSKNT") != -1:
            return [0]*len(self.hrp2ResetPose())
        elif self.ROBOT_NAME.find("HRP2JSK") != -1:
            ret=[0]*len(self.hrp2ResetPose())
            ret[31]=-0.261799
            ret[23]=0.261799
            return ret
        else:
            return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, 1.0472, 0.0, 0.0, 0.0, -1.5708, 0.0, 0.0, 0.0, -1.0472]

    def setStAbcParameters (self):
        if self.ROBOT_NAME == "HRP2JSKNT":
            self.setStAbcParametershrp2016c()
        elif self.ROBOT_NAME == "HRP2JSKNTS":
            self.setStAbcParametershrp2017c() # for hrp2017
        elif self.ROBOT_NAME == "HRP2JSK":
            self.setStAbcParametershrp2007c() 

    # for eefm Stabilizer, hrp2017, new
    def setStAbcParametershrp2017c(self):
        # ABC parameters
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets = [[0.015, -0.01, 0], [0.015, 0.01, 0], [0, 0, 0], [0, 0, 0]]
        #abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets = [[0.01, 0.01, 0], [0.01, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.ik_mode = OpenHRP.AutoBalancerService.FULLBODY
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp=self.abc_svc.getStabilizerParam()
        stp.st_algorithm=OpenHRP.AutoBalancerService.EEFMQP
        #   eefm st params
        stp.eefm_body_attitude_control_again=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        # EEFM parameters for 4 limbs
        #stp.eefm_rot_damping_gain = [[20*1.6, 20*1.6, 1e5]]*4
        #stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.0]]*4
        stp.eefm_rot_damping_gain = [[35, 35, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3700*1.0]]*4
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_swing_pos_damping_gain=stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain=stp.eefm_rot_damping_gain[0]
        stp.eefm_use_swing_damping=True
        stp.eefm_wrench_alpha_blending = 0.6
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.0, 0.0]
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.065
        #stp.eefm_leg_front_margin=0.140
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.062
        tmp_leg_outside_margin=0.062
        tmp_leg_front_margin=0.130
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.AutoBalancerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
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
        # stp.emergency_check_mode=OpenHRP.AutoBalancerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        # for swing
        stp.eefm_swing_pos_spring_gain = [[5]*3, [5]*3, [0]*3, [0]*3]
        stp.eefm_swing_rot_spring_gain = [[5]*3, [5]*3, [0]*3, [0]*3]
        stp.use_zmp_truncation = True
        stp.detection_time_to_air = 1.0
        self.abc_svc.setStabilizerParam(stp)
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=0.8
        gg.default_double_support_ratio=0.1
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset= gg.default_step_time * (1.0 - gg.default_double_support_ratio) * 0.3
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        #  Orbit time parameters for delayhoffarbib (simultaneous xy and z landing)
        #gg.swing_trajectory_final_distance_weight=3.0
        #gg.swing_trajectory_time_offset_xy2z=0.0
        #  Orbit time parameters for delayhoffarbib (xy is faster than z)
        gg.swing_trajectory_final_distance_weight=1.5
        gg.swing_trajectory_time_offset_xy2z=0.1 # [s]
        #
        # gg.default_orbit_type = OpenHRP.AutoBalancerService.CYCLOIDDELAY
        gg.default_orbit_type = OpenHRP.AutoBalancerService.RECTANGLE
        gg.toe_pos_offset_x = 1e-3*142.869;
        gg.heel_pos_offset_x = 1e-3*-105.784;
        gg.toe_zmp_offset_x = 1e-3*79.411;
        gg.heel_zmp_offset_x = 1e-3*-105.784;
        gg.use_toe_joint = False
        gg.optional_go_pos_finalize_footstep_num = 1
        gg.overwritable_footstep_index_offset = 1
        gg.leg_marign = [0.13, 0.095, 0.062, 0.062]
        gg.safe_leg_margin = [0.07, 0.055, 0.057, 0.057]
        gg.stride_limitation_for_circle_type = [0.15, 0.3, 15, 0.1, 0.138]
        gg.overwritable_stride_limitation = [0.35, 0.45, 0, 0.35, 0.128]
        gg.margin_time_ratio = 0.25
        gg.min_time_mgn = 0.3
        gg.use_disturbance_compensation = True
        gg.dc_gain = 1e-3
        gg.dcm_offset = 0.02
        gg.modify_footsteps = True
        gg.use_act_states = True
        gg.stride_limitation_type = OpenHRP.AutoBalancerService.CIRCLE
        gg.min_time = 0.7
        gg.overwritable_max_time = 1.5
        gg.is_interpolate_zmp_in_double = True
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setStAbcParametershrp2016c (self):
        # ABC parameters
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets = [[0.010, 0.01, 0], [0.010, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        #abcp.default_zmp_offsets = [[0.01, 0.035, 0], [0.01, -0.035, 0], [0, 0, 0], [0, 0, 0]]
        abcp.ik_mode = OpenHRP.AutoBalancerService.FULLBODY
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp=self.abc_svc.getStabilizerParam()
        stp.st_algorithm=OpenHRP.AutoBalancerService.EEFMQP
        #   eefm st params
        stp.eefm_body_attitude_control_gain=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        # EEFM parameters for 4 limbs
        stp.eefm_rot_damping_gain = [[20*1.1, 20*1.1, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.1]]*4
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_swing_pos_damping_gain=stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain=stp.eefm_rot_damping_gain[0]
        stp.eefm_use_swing_damping=True
        stp.eefm_wrench_alpha_blending = 0.75
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.0, 0.0]
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.065
        #stp.eefm_leg_front_margin=0.140
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.062
        tmp_leg_outside_margin=0.062
        tmp_leg_front_margin=0.130
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.AutoBalancerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
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
        stp.emergency_check_mode=OpenHRP.AutoBalancerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        # for swing
        stp.eefm_swing_pos_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        stp.eefm_swing_rot_spring_gain = [[1]*3, [1]*3, [0]*3, [0]*3]
        stp.use_zmp_truncation = True
        stp.detection_time_to_air = 1.0
        self.abc_svc.setStabilizerParam(stp)
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=1.1
        gg.default_double_support_ratio=0.32
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset=0.2
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        #  Orbit time parameters for delayhoffarbib (simultaneous xy and z landing)
        #gg.swing_trajectory_final_distance_weight=3.0
        #gg.swing_trajectory_time_offset_xy2z=0.0
        #  Orbit time parameters for delayhoffarbib (xy is faster than z)
        gg.swing_trajectory_final_distance_weight=1.5
        gg.swing_trajectory_time_offset_xy2z=0.1 # [s]
        #
        gg.default_orbit_type = OpenHRP.AutoBalancerService.RECTANGLE
        gg.toe_pos_offset_x = 1e-3*142.869;
        gg.heel_pos_offset_x = 1e-3*-105.784;
        gg.toe_zmp_offset_x = 1e-3*79.411;
        gg.heel_zmp_offset_x = 1e-3*-105.784;
        gg.use_toe_joint = False
        gg.optional_go_pos_finalize_footstep_num = 1
        gg.overwritable_footstep_index_offset = 1
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setStAbcParametershrp2007c (self):
        # ABC parameters
        abcp=self.abc_svc.getAutoBalancerParam()[1]
        #abcp.default_zmp_offsets = [[0.015, 0.01, 0], [0.015, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets = [[0.010, 0.01, 0], [0.010, -0.01, 0], [0, 0, 0], [0, 0, 0]]
        #abcp.default_zmp_offsets = [[0.01, 0.035, 0], [0.01, -0.035, 0], [0, 0, 0], [0, 0, 0]]
        abcp.ik_mode = OpenHRP.AutoBalancerService.FULLBODY
        self.abc_svc.setAutoBalancerParam(abcp)
        # ST parameters
        stp=self.abc_svc.getStabilizerParam()
        stp.st_algorithm=OpenHRP.AutoBalancerService.EEFMQP
        #   eefm st params
        stp.eefm_body_attitude_control_gain=[1.5, 1.5]
        stp.eefm_body_attitude_control_time_const=[10000, 10000]
        # EEFM parameters for 4 limbs
        stp.eefm_rot_damping_gain = [[20*1.1, 20*1.1, 1e5]]*4
        stp.eefm_pos_damping_gain = [[3500*50, 3500*50, 3500*1.1]]*4
        stp.eefm_rot_time_const = [[1.5, 1.5, 1.5]]*4
        stp.eefm_pos_time_const_support = [[1.5, 1.5, 1.5]]*4
        stp.eefm_swing_pos_damping_gain=stp.eefm_pos_damping_gain[0]
        stp.eefm_swing_rot_damping_gain=stp.eefm_rot_damping_gain[0]
        stp.eefm_use_swing_damping=True
        stp.eefm_wrench_alpha_blending = 0.7
        stp.eefm_pos_time_const_swing=0.08
        stp.eefm_pos_transition_time=0.01
        stp.eefm_pos_margin_time=0.02
        stp.eefm_zmp_delay_time_const=[0.0, 0.0]
        stp.eefm_cogvel_cutoff_freq=6.0
        #   mechanical foot edge
        #stp.eefm_leg_inside_margin=0.07
        #stp.eefm_leg_front_margin=0.135
        #stp.eefm_leg_rear_margin=0.105
        #   margined foot edge
        tmp_leg_inside_margin=0.062
        tmp_leg_outside_margin=0.062
        tmp_leg_front_margin=0.125
        tmp_leg_rear_margin=0.095
        stp.eefm_leg_inside_margin=tmp_leg_inside_margin
        stp.eefm_leg_outside_margin=tmp_leg_outside_margin
        stp.eefm_leg_front_margin=tmp_leg_front_margin
        stp.eefm_leg_rear_margin=tmp_leg_rear_margin
        rleg_vertices = [OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_inside_margin])]
        lleg_vertices = [OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, tmp_leg_outside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[tmp_leg_front_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, -1*tmp_leg_inside_margin]),
                         OpenHRP.AutoBalancerService.TwoDimensionVertex(pos=[-1*tmp_leg_rear_margin, tmp_leg_outside_margin])]
        rarm_vertices = rleg_vertices
        larm_vertices = lleg_vertices
        stp.eefm_support_polygon_vertices_sequence = map (lambda x : OpenHRP.AutoBalancerService.SupportPolygonVertices(vertices=x), [rleg_vertices, lleg_vertices, rarm_vertices, larm_vertices])
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
        stp.emergency_check_mode=OpenHRP.AutoBalancerService.CP;
        stp.cp_check_margin=[50*1e-3, 45*1e-3, 0, 100*1e-3];
        # for swing
        stp.eefm_swing_pos_spring_gain = [[5]*3, [5]*3, [0]*3, [0]*3]
        stp.eefm_swing_rot_spring_gain = [[5]*3, [5]*3, [0]*3, [0]*3]
        stp.use_zmp_truncation = True
        stp.detection_time_to_air = 1.0
        self.abc_svc.setStabilizerParam(stp)
        # GG parameters
        gg=self.abc_svc.getGaitGeneratorParam()[1]
        gg.default_step_time=0.8
        gg.default_double_support_ratio=0.1
        #gg.swing_trajectory_delay_time_offset=0.35
        gg.swing_trajectory_delay_time_offset= gg.default_step_time * (1.0 - gg.default_double_support_ratio) * 0.3
        gg.stair_trajectory_way_point_offset=[0.03, 0.0, 0.0]
        #  Orbit time parameters for delayhoffarbib (simultaneous xy and z landing)
        #gg.swing_trajectory_final_distance_weight=3.0
        #gg.swing_trajectory_time_offset_xy2z=0.0
        #  Orbit time parameters for delayhoffarbib (xy is faster than z)
        gg.swing_trajectory_final_distance_weight=1.5
        gg.swing_trajectory_time_offset_xy2z=0.1 # [s]
        #
        gg.default_orbit_type = OpenHRP.AutoBalancerService.RECTANGLE
        gg.toe_pos_offset_x = 1e-3*137.525;
        gg.heel_pos_offset_x = 1e-3*-106.925;
        gg.toe_zmp_offset_x = 1e-3*137.525;
        gg.heel_zmp_offset_x = 1e-3*-106.925;
        gg.optional_go_pos_finalize_footstep_num = 1
        gg.overwritable_footstep_index_offset = 1
        gg.leg_marign = [0.13, 0.095, 0.062, 0.062]
        gg.safe_leg_margin = [0.07, 0.055, 0.057, 0.057]
        gg.stride_limitation_for_circle_type = [0.15, 0.3, 15, 0.1, 0.138]
        gg.overwritable_stride_limitation = [0.35, 0.45, 0, 0.35, 0.128]
        gg.margin_time_ratio = 0.25
        gg.min_time_mgn = 0.3
        gg.use_disturbance_compensation = True
        gg.dc_gain = 1e-3
        gg.dcm_offset = 0.02
        gg.modify_footsteps = True
        gg.use_act_states = True
        gg.stride_limitation_type = OpenHRP.AutoBalancerService.CIRCLE
        gg.min_time = 0.7
        gg.overwritable_max_time = 1.5
        gg.is_interpolate_zmp_in_double = True
        self.abc_svc.setGaitGeneratorParam(gg)
        # Estop
        esp=self.es_svc.getEmergencyStopperParam()[1]
        esp.default_recover_time=10.0 # [s]
        esp.default_retrieve_time=1.0 # [s]
        self.es_svc.setEmergencyStopperParam(esp)

    def setResetPose(self):
        self.seq_svc.setJointAngles(self.hrp2ResetPose(), 5.0)

    def setResetManipPose(self):
        self.seq_svc.setJointAngles(self.hrp2ResetManipPose(), 5.0)

    def setInitPose(self):
        self.seq_svc.setJointAngles(self.hrp2InitPose(), 5.0)

    def loadForceMomentOffsetFile (self):
        import rospkg
        if self.ROBOT_NAME == "HRP2JSKNT":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_HRP2JSKNT")
        elif self.ROBOT_NAME == "HRP2JSKNTS":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_HRP2JSKNTS")
        elif self.ROBOT_NAME == "HRP2JSK":
            self.rmfo_svc.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_thumb_60deg_HRP2JSK")
        else:
            print "No force moment offset file"

    def hrp3HandResetPose(self):
        self.hc_svc.setJointAngles([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 2)
        self.hc_svc.waitInterpolation()

    def hrp3HandGraspPose(self):
        self.hc_svc.setJointAngles([77.9709, -11.4732, 8.28742, 0.0, 106.185, 86.0974, 77.9709, -11.4732, 8.28742, 0.0, 106.185, 86.0974], 2)
        self.hc_svc.waitInterpolation()

    def hrp3HandHookPose(self):
        self.hc_svc.setJointAngles([90.0, 90.0, 0.0, 10.0, -20.0, -20.0, 90.0, 90.0, 0.0, 10.0, -20.0, -20.0], 2)
        self.hc_svc.waitInterpolation()

    def hrp3HandCalib(self):
        self.hc_svc.handJointCalib()

    def hrp3HandServoOn(self):
        self.hc_svc.handServoOn()

    def hrp3HandServoOff(self):
        self.hc_svc.handServoOff()

    def hrp3HandReconnect(self):
        self.hc_svc.handReconnect()

    def __init__(self, robotname=""):
        self.ROBOT_NAME = robotname
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()
