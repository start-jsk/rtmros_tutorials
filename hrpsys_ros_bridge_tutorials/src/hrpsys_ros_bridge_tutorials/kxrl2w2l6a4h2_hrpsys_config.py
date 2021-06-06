#!/usr/bin/env python

from urata_hrpsys_config import *

if __name__ == '__main__':
    hcf = URATAHrpsysConfigurator("KXRL2W2L6A4H2")
    if len(sys.argv) > 2 :
        hcf.init(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 1 :
        hcf.init(sys.argv[1], sys.argv[2])
    else :
        hcf.init()
