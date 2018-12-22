#!/usr/bin/env python

from jaxon_hrpsys_config import *

class JAXON_REDHrpsysConfigurator(JAXONHrpsysConfigurator):
    def __init__(self):
        URATAHrpsysConfigurator.__init__(self, "JAXON_RED")

    def setDefaultForceMomentOffset(self):
        import rospkg
        self.loadForceMomentOffsetParams(rospkg.RosPack().get_path('hrpsys_ros_bridge_tutorials')+"/models/hand_force_calib_offset_JAXON_RED")

    def setDefaultABCParameters(self):
        abcp = self.getABCParameters()
        #abcp.default_zmp_offsets=[[0.015, 0.0, 0.0], [0.015, 0.0, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.default_zmp_offsets=[[0.0, 0.01, 0.0], [0.0, -0.01, 0.0], [0, 0, 0], [0, 0, 0]]
        abcp.move_base_gain=0.8
        self.setABCParameters(abcp)

    def setDefaultSTParameters(self):
        stp=self.getSTParameters()
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
        stp.eefm_cogvel_cutoff_freq = 4.0
        stp.eefm_k1=[-1.48412,-1.48412]
        stp.eefm_k2=[-0.486727,-0.486727]
        stp.eefm_k3=[-0.198033,-0.198033]
        self.setSTParameters(stp)
        self.setJAXONFootMarginParam(foot="KAWADA")

if __name__ == '__main__':
    hcf = JAXON_REDHrpsysConfigurator()
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
