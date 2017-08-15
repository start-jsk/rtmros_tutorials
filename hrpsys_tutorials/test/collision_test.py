#!/usr/bin/env python

try:
    from hrpsys.hrpsys_config import *
    from hrpsys_tutorials import jaxon_client as jaxon
    import OpenHRP
except:
    print "import without hrpsys"
    import rtm
    from rtm import *
    from OpenHRP import *
    import waitInput
    from waitInput import *
    import socket
    import time

def vector_equal_eps (vec1, vec2, eps=1e-5):
    if len(vec1) == len(vec2):
        for e1, e2 in zip(vec1, vec2):
            if abs(e1 - e2) > eps:
                return False
        return True
    else:
        return False

def init ():
    global hcf, init_pose, collision_pose, hrpsys_version, fout

    fout = open('result.txt', 'w')
    fout.write('Condition\tComp Time\tRecov Time\n')

    hcf = jaxon.JaxonConfigurator()
    hcf.init("JAXON(Robot)0")

    init_pose = [0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6981316888888882, -0.34906584444444466, -0.08726646111111103, -1.3962633777777764, 0.0, 0.0, -0.3490658444444441, 0.0, 0.6981316888888882, 0.34906584444444466, 0.08726646111111103, -1.3962633777777764, 0.0, 0.0, -0.3490658444444441, 0.0, 0.0, 2.775557514216984e-17, 1.387778757108492e-17]
    collision_pose = [
    [0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.3962633777777764, 0.6981316888888882, -0.34906584444444466, -0.08726646111111103, -1.3962633777777764, 0.0, 0.0, -0.3490658444444441, -1.3962633777777764, 0.6981316888888882, 0.34906584444444466, 0.08726646111111103, -1.3962633777777764, 0.0, 0.0, -0.3490658444444441, 6.93889378554246e-18, -6.93889378554246e-18, 0.0, 1.387778757108492e-17],
    [0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0943950666666677, 0.6981316888888882, -0.34906584444444466, -0.08726646111111103, -1.0471975333333332, -0.8726646111111127, 0.0, -0.3490658444444441, -0.872664611111111, 0.6981316888888882, 0.34906584444444466, 0.08726646111111103, -1.396263377777778, 0.0, 0.0, -0.3490658444444441, -1.387778757108492e-17, 1.734723446385615e-18, -1.387778757108492e-17, -1.734723446385615e-18],
    [0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, -0.3490658444444441, 0.6981316888888882, -0.3490658444444441, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 2.0943950666666677, 0.6981316888888882, -0.34906584444444466, -0.08726646111111103, -1.0471975333333332, -0.8726646111111127, 0.0, -0.3490658444444441, -1.1344639944444443, 0.6981316888888882, 0.34906584444444466, 0.08726646111111103, -1.7453292222222228, -0.17453292222222205, -0.17453292222222205, -0.3490658444444441, -1.387778757108492e-17, 1.734723446385615e-18, -1.387778757108492e-17, -1.734723446385615e-18]
    ]

def printCollisionState(cs):
    print >> sys.stderr, "Collision State:\n"
    print >> sys.stderr, "Time: %f" % cs.time
    print >> sys.stderr, "Computation time: %f" % cs.computation_time
    print >> sys.stderr, "Safe Posture: %s" % cs.safe_posture
    print >> sys.stderr, "Recover time: %f" % cs.recover_time

def outputCollisionState(cs, condition):
    s = "%d\t%f\t%f\n" % (condition, cs.computation_time, cs.recover_time)
    fout.write(s)

def demoCollisionCheckFail (index):
    print >> sys.stderr, "Index: %d" % index
    print >> sys.stderr, "CollisionCheck in fail pose"
    hcf.seq_svc.setJointAngles(collision_pose[index], 3.0);
    hcf.waitInterpolation();
    cs = hcf.co_svc.getCollisionStatus()[1]
    if not cs.safe_posture:
        outputCollisionState(cs, index)
        print >> sys.stderr, "  => Successfully stop fail pose"
    assert((not cs.safe_posture) is True)
    hcf.seq_svc.setJointAngles(init_pose, 3.0);
    hcf.waitInterpolation();
    cs = hcf.co_svc.getCollisionStatus()[1]
    if cs.safe_posture:
        outputCollisionState(cs, index)
        print >> sys.stderr, "  => Successfully return to safe pose"
    assert(cs.safe_posture is True)

def demo():
    init()
    for i in range(0, len(collision_pose)):
        demoCollisionCheckFail(i)

if __name__ == '__main__':
    demo()
