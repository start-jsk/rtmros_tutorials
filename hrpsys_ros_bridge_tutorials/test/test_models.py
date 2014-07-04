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
        self.dae_url = rospkg.RosPack().get_path("hrpsys_ros_bridge_tutorials")+"/models/"+dae_file
        self.wrl_binfo = self.ml.getBodyInfo(self.wrl_url)
        self.dae_binfo = self.ml.getBodyInfo(self.dae_url)
        self.wrl_links = self.wrl_binfo._get_links()
        self.dae_links = self.dae_binfo._get_links()

    def test_STARO_models(self):
        self.checkModels(os.environ['CVSDIR']+"/euslib/rbrain/staro/STAROmain.wrl", "STARO.dae")

if __name__ == '__main__':
    rostest.run(PKG, NAME, TestModelLoaderORR, sys.argv)
