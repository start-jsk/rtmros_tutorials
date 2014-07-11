#!/usr/bin/env python
import rospkg
import os
import sys

sys.path.append(rospkg.RosPack().get_path("openhrp3")+"/test")

from test_modelloader import *

PKG = 'hrpsys_ros_bridge_tutorials'
NAME = 'hrpsys_ros_bridge_tutorials_model_test'

class TestModelLoaderORR(TestModelLoaderBase):
    def loadFiles(self, wrl_file, dae_file):
        self.wrl_url = wrl_file
        self.dae_url = dae_file
        self.wrl_binfo = self.ml.getBodyInfo(self.wrl_url)
        self.dae_binfo = self.ml.getBodyInfo(self.dae_url)
        self.wrl_links = self.wrl_binfo._get_links()
        self.dae_links = self.dae_binfo._get_links()

    def test_STARO_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"STARO.dae"
        if os.path.exists(dae_file):
            self.checkModels(os.environ['CVSDIR']+"/euslib/rbrain/staro/STAROmain.wrl", dae_file)
        else:
            return True

    def test_URATALEG_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"URATALEG.dae"
        if os.path.exists(dae_file):
            self.checkModels(os.environ['CVSDIR']+"/euslib/rbrain/urataleg/URATALEGmain.wrl", dae_file)
        else:
            return True

    def test_HRP2JSK_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"HRP2JSK.dae"
        if os.path.exists(dae_file):
            self.checkModels(os.environ['CVSDIR']+"/OpenHRP/etc/HRP2JSK_for_OpenHRP3/HRP2JSKmain.wrl", dae_file)
        else:
            return True

    def test_HRP2JSKNT_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"HRP2JSKNT.dae"
        if os.path.exists(dae_file):
            self.checkModels(os.environ['CVSDIR']+"/OpenHRP/etc/HRP2JSKNT_for_OpenHRP3/HRP2JSKNTmain.wrl", dae_file)
        else:
            return True

    def test_HRP2JSKNTS_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"HRP2JSKNTS.dae"
        if os.path.exists(dae_file):
            self.checkModels(os.environ['CVSDIR']+"/OpenHRP/etc/HRP2JSKNTS_for_OpenHRP3/HRP2JSKNTSmain.wrl", dae_file)
        else:
            return True

    def test_HRP4R_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"HRP4R.dae"
        if os.path.exists(dae_file):
            self.checkModels(os.environ['CVSDIR']+"/OpenHRP/etc/HRP4R/HRP4Rmain.wrl", dae_file)
        else:
            return True

    def test_GR001_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"GR000.dae"
        if os.path.exists(dae_file):
            wrl_file = rospkg.RosPack().get_path("choreonoid++")+"/share/choreonoid-1.3/model/GR001/GR001.wrl"
            self.checkModels(wrl_file, dae_file)
        else:
            return True

    def test_HRP4C_models(self):
        dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"HRP4C.dae"
        if os.path.exists(dae_file):
            wrl_file = rospkg.RosPack().get_path("hrpsys")+"/share/hrpsys/samples/HRP4C/HRP4Cmain.wrl"
            self.checkModels(wrl_file, dae_file)
        else:
            return True

    ## no wrl file
    #def test_kawada_hiro_nx_models(self):
    #def test_DARWIN_models(self):
    #def test_PR2_models(self):

    # def test_SmartPal_models(self):
    #     dae_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"YaskawaSmartPal5.dae"
    #     wrl_file = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+"YaskawaSmartPal5.wrl"
    #     if os.path.exists(dae_file):
    #         self.checkModels(wrl_file, dae_file)
    #     else:
    #         return True

if __name__ == '__main__':
    rostest.run(PKG, NAME, TestModelLoaderORR, sys.argv)
