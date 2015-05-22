#!/usr/bin/env python

from hrp2_hrpsys_config import *

if __name__ == '__main__':
    hcf = JSKHRP2HrpsysConfigurator("HRP2JSK")
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1])
    else :
        hcf.init()
