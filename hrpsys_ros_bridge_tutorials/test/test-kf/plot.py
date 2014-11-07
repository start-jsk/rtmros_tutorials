#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import matplotlib.pyplot, numpy

def parse_args():
    parser = argparse.ArgumentParser(description='plot data from rosbag')
    parser.add_argument('-f', type=str, help='input file', metavar='file', default='/tmp/kuroiwa.dat')
    return parser.parse_args()

def process(args):
    tm = []
    ypr = [[], [], []]
    raw = [[], [], []]
    order = ['yaw', 'pitch', 'roll']
    raw_order = ['acceleration', 'gyro', 'position']
    fig, (axes_array) = matplotlib.pyplot.subplots(3, 2, sharex=True, sharey=False)
    with open(args.f) as f:
        for line in f:
            tmp = [float(x) for x in line.rstrip().split(' ')]
            tm.append(tmp[0])
            ypr[0].append(tmp[1:(10+1):3]) # 1, 4, 7, 10
            ypr[1].append(tmp[2:(11+1):3]) # 2, 5, 8, 11
            ypr[2].append(tmp[3:(12+1):3]) # 3, 6, 9, 12
            raw[0].append(tmp[13:(15+1)])  # 13, 14, 15
            raw[1].append(tmp[16:(18+1)])  # 16, 17, 18
            raw[2].append(tmp[19:(21+1)])  # 19, 20, 21
    for i in range(len(ypr)):
        axes_array[i][0].plot(tm, [hoge[0] for hoge in ypr[i]], 'g.', label='kf_rpy')
        axes_array[i][0].plot(tm, [hoge[1] for hoge in ypr[i]], 'r.', label='FowardKinematics')
        # axes_array[i].plot(tm, [hoge[1] for hoge in ypr[i]], 'b.', label='imumsg')
        # axes_array[i].plot(tm, [hoge[1] for hoge in ypr[i]], 'y.', label='imucoords')
        axes_array[i][0].set_title(order[i])
        axes_array[i][0].legend(loc=0)
    for i in range(len(raw)):
        axes_array[i][1].plot(tm, [hoge[0] for hoge in raw[i]], 'r.', label='x')
        axes_array[i][1].plot(tm, [hoge[1] for hoge in raw[i]], 'g.', label='y')
        axes_array[i][1].plot(tm, [hoge[2] for hoge in raw[i]], 'b.', label='z')
        axes_array[i][1].set_title(raw_order[i])
        axes_array[i][1].legend(loc=0)
    axes_array[2][0].set_xlabel('t [s]')
    axes_array[1][0].set_ylabel('[deg]')
    matplotlib.pyplot.show()


def main():
    args = parse_args()
    process(args)

if __name__ == '__main__':
    main()
