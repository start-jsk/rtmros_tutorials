
from hrpsys import rtm
from hrpsys.hrpsys_config import *

class JaxonConfigurator(HrpsysConfigurator):

    Groups = [['torso', ['CHEST_JOINT0']], 
    ['head', ['HEAD_JOINT0', 'HEAD_JOINT1']], 
    ['rarm', ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5', 'RARM_JOINT6', 'RARM_JOINT7']],
    ['larm', ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5', 'LARM_JOINT6', 'LARM_JOINT7']]]

    rtclist = [
            ['seq', "SequencePlayer"],
            ['sh', "StateHolder"],
            ['fk', "ForwardKinematics"],
            ['tf', "TorqueFilter"],
            ['kf', "KalmanFilter"],
            ['vs', "VirtualForceSensor"],
            ['rmfo', "RemoveForceSensorLinkOffset"],
            ['octd', "ObjectContactTurnaroundDetector"],
            ['es', "EmergencyStopper"],
            ['rfu', "ReferenceForceUpdater"],
            ['ic', "ImpedanceController"],
            ['abc', "AutoBalancer"],
            ['st', "Stabilizer"],
            ['co', "CollisionDetector"],
            ['tc', "TorqueController"],
            ['te', "ThermoEstimator"],
            ['hes', "EmergencyStopper"],
            ['el', "SoftErrorLimiter"],
            ['tl', "ThermoLimiter"],
            ['bp', "Beeper"],
            ['acf', "AccelerationFilter"],
            ['log', "DataLogger"]
            ]

    def init(self, robotname="", url=""):
        HrpsysConfigurator.Groups = self.Groups
        HrpsysConfigurator.init(self, robotname, url)

    def goPos(self, x, y, th):
        '''!@brief Walk to the goal position and orientation. Returns without waiting for whole steps to be executed.
        @param i_x[m], i_y[m], and i_th[deg] are goal x-y-position and z-orientation from the current mid-coords of right foot and left foot.
        @return true if set successfully, false otherwise'''
        self.abc_svc.goPos(x, y, th)

    def goVelocity(self, vx, vy, vth):
        '''!@brief Walk at the desired velocity. If the robot is stopping, the robot starts stepping. Returns without waiting for whole steps to be executed.
        @param i_vx[m/s], i_vy[m/s], and i_vth[deg/s] are velocity in the current mid-coords of right foot and left foot.
        @return true if set successfully, false otherwise'''
        self.abc_svc.goVelocity(vx, vy, vth)

    def goStop(self):
        '''!@brief Stop stepping.
        @param
        @return true if set successfully, false otherwise'''
        self.abc_svc.goStop()

    def getRTCList(self):
        return self.rtclist

