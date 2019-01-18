#!/usr/bin/env python

pkg = 'hrpsys'
import imp
imp.find_module(pkg)

from hrpsys.hrpsys_config import *
import OpenHRP
import math

class URATAHrpsysConfigurator(HrpsysConfigurator):
    """
    Abstract class for JSK robot configuration.
    Please inherit this class to specify robot or environmnet-dependent class.
    """

    def __init__(self, robotname=""):
        self.ROBOT_NAME = robotname
        HrpsysConfigurator.__init__(self)
        self.defJointGroups()

    def getRTCList (self):
        return self.getRTCListUnstable()

    def init (self, robotname="Robot", url=""):
        HrpsysConfigurator.init(self, robotname, url)
        self.setDefaultHrpsysParameters()

    # Wrapper functions to get hrpsys parameters
    def getKFParameters(self):
        return self.kf_svc.getKalmanFilterParam()[1]

    def getESParameters(self):
        return self.es_svc.getEmergencyStopperParam()[1]

    def getICParameters(self, limb):
        return self.ic_svc.getImpedanceControllerParam(limb)[1]

    def getABCParameters(self):
        return self.abc_svc.getAutoBalancerParam()[1]

    def getGaitGeneraterParameters(self):
        return self.abc_svc.getGaitGeneratorParam()[1]

    def getSTParameters(self):
        return self.st_svc.getParameter()

    # Wrapper functions to set hrpsys parameters
    def setKFParameters(self, param):
        self.kf_svc.setKalmanFilterParam(param)

    def loadForceMomentOffsetParams(self, file_path):
        self.rmfo_svc.loadForceMomentOffsetParams(file_path)

    def setESParameters(self, param):
        self.es_svc.setEmergencyStopperParam(param)

    def setICParameters(self, limb, param):
        self.ic_svc.setImpedanceControllerParam(limb, param)

    def setABCParameters(self, param):
        self.abc_svc.setAutoBalancerParam(param)

    def setGaitGeneraterParameters(self, param):
        self.abc_svc.setGaitGeneratorParam(param)

    def setSTParameters(self, param):
        self.st_svc.setParameter(param)

    def setServoErrorLimit(self, joint_name, limit):
        self.el_svc.setServoErrorLimit(joint_name, limit)

    # Functions to set default hrpsys parameters for each robot
    def setDefaultHrpsysParameters(self):
        if self.kf:
            self.setDefaultKFParameters()
        if self.rmfo:
            self.setDefaultForceMomentOffset()
        if self.es:
            self.setDefaultESParameters()
        if self.ic:
            self.setDefaultICParameters()
        if self.abc:
            self.setDefaultABCParameters()
            self.setDefaultGaitGeneraterParameters()
        if self.st:
            self.setDefaultSTParameters()
        if self.el:
            self.setDefaultServoErrorLimit()

    def setDefaultKFParameters(self):
        print("setDefaultKFParameters is not implemented")

    def setDefaultForceMomentOffset(self):
        print("No force moment offset file")

    def setDefaultESParameters(self):
        print("setDefaultESParameters is not implemented")

    def setDefaultICParameters(self):
        print("setDefaultICParameters is not implemented")

    def setDefaultABCParameters(self):
        print("setDefaultABCParameters is not implemented")

    def setDefaultGaitGeneraterParameters(self):
        print("setDefaultGaitGeneraterParameters is not implemented")

    def setDefaultSTParameters(self):
        print("setDefaultSTParameters is not implemented")

    def setDefaultServoErrorLimit(self):
        pass

    # Basic set of poses
    def setJointAnglesRadian(self, angle_vector, time=10.0):
        if not angle_vector:
            print("Not implemented yet")
            return
        self.seq_svc.setJointAngles(angle_vector, time)

    def resetPose(self):
        return []

    def setResetPose(self, time=5.0):
        self.setJointAnglesRadian(self.resetPose(), time)

    def initPose(self):
        return [0] * len(self.resetPose())

    def setInitPose(self, time=10.0):
        self.setJointAnglesRadian(self.initPose(), time)

    def resetManipPose(self):
        return []

    def setResetManipPose(self, time=10.0):
        self.setJointAnglesRadian(self.resetManipPose(), time)

    def collisionFreeInitPose(self):
        return []

    def setCollisionFreeInitPose(self, time=10.0):
        self.setJointAnglesRadian(self.collisionFreeInitPose(), time)

    def resetLandingPose(self):
        return []

    def setResetLandingPose(self, time=5.0):
        self.setJointAnglesRadian(self.resetLandingPose(), time)
