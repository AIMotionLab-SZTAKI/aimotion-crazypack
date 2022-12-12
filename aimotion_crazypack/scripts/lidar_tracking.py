#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory


def goCircles2():
    TIMESCALE = 0.6
    for cf in allcfs.crazyflies:
        cf.startTrajectory(0, timescale=TIMESCALE, reverse=False)
    timeHelper.sleep(circle0.duration*TIMESCALE)


if __name__ == "__main__":

    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    circle0 = uav_trajectory.Trajectory()
    circle0.loadcsv("csv/circle_traj_0.csv")

    allcfs.crazyflies[0].uploadTrajectory(0, 0, circle0)

    # start flying!
    initHeight = 0.4
    numDrones = len(allcfs.crazyflies)
    N = 10
    T = 63
    r = 0.4
    xinit = [r * np.cos(2 * np.pi * k / numDrones) for k in range(numDrones)]
    yinit = [r * np.sin(2 * np.pi * k / numDrones) for k in range(numDrones)]
    x = np.array([0.3, 0.3, 0.3])
    y = np.array([0.5, 0, -0.5])
    k = 0

    allcfs.takeoff(targetHeight=initHeight+0.2, duration=2)
    timeHelper.sleep(2)
    # for cf in allcfs.crazyflies:
    #     cf.goTo(np.array([xinit[k], yinit[k], initHeight + 0.2]), 0, 2)
    #     k = k+1
    # k = 0
    # timeHelper.sleep(3)
    #
    # goCircles2()
    # timeHelper.sleep(1)

    d = 3

    for _ in range(3):
        for cf in allcfs.crazyflies:
            cf.goTo(np.array([-0.9, 0, 1.4]), 0, d)
        timeHelper.sleep(d)

        for cf in allcfs.crazyflies:
            cf.goTo(np.array([0, -1.2, 0.3]), 0, d)
        timeHelper.sleep(d)

        for cf in allcfs.crazyflies:
            cf.goTo(np.array([1.1, 0, 1.2]), 0, d)
        timeHelper.sleep(d)

        for cf in allcfs.crazyflies:
            cf.goTo(np.array([0, 1.2, 0.3]), 0, d)
        timeHelper.sleep(d)

    # timeHelper.sleep(2)
    allcfs.land(targetHeight=0.06, duration=3)
