#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

def flipInitParams(cf):
    # send the parameters
    # cf.uploadTrajectory(0, 0, traj1)
    cf.setParams(params)
    cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
    cf.setParam('stabilizer/controller', 4)  # PID controller
    cf.setParam('ctrlFlip/controller_type', 1)
    cf.setParam('ctrlFlip/z0', 2.0)
    cf.setParam('ctrlFlip/rec_sp_z', 0.6)

def goCircles(N, T, h, r, numDrones):
    # for k in range(numDrones):
    #     allcfs.crazyflies[k].goTo(
    #         np.array([r * np.cos(2 * np.pi * k / numDrones), r * np.sin(2 * np.pi * k / numDrones), h]), 0, 3)
    # timeHelper.sleep(3)
    for t in range(T):
        for k in range(numDrones):
            allcfs.crazyflies[k].goTo(np.array(
                [r * np.cos(t / N + 2 * np.pi * k / numDrones), r * np.sin(t / N + 2 * np.pi * k / numDrones),
                 h + t / T * 2 / 3]), 0,
                0.1)
        timeHelper.sleepForRate(10)
    timeHelper.sleep(1)
    for t in range(T):
        for k in range(numDrones):
            allcfs.crazyflies[k].goTo(np.array(
                [r * np.cos(-t / N + 2 * np.pi * k / numDrones), r * np.sin(-t / N + 2 * np.pi * k / numDrones),
                 h + 2 / 3 - t / T * 2 / 3]), 0,
                0.1)
        timeHelper.sleepForRate(10)

def goCircles2():
    TIMESCALE = 0.5
    for cf in allcfs.crazyflies:
        cf.startTrajectory(1, timescale=TIMESCALE, reverse=False)
    timeHelper.sleep(circle0.duration*TIMESCALE)

if __name__ == "__main__":

    # Adjust flip parameters (these are empirical)
    T0 = 0.45
    T1 = 0.9

    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # traj1 = uav_trajectory.Trajectory()
    # traj1.loadcsv("flip_traj.csv")

    circle0 = uav_trajectory.Trajectory()
    circle0.loadcsv("csv/circle_traj_0.csv")
    circle1 = uav_trajectory.Trajectory()
    circle1.loadcsv("csv/circle_traj_1.csv")
    circle2 = uav_trajectory.Trajectory()
    circle2.loadcsv("csv/circle_traj_2.csv")

    allcfs.crazyflies[0].uploadTrajectory(1, 0, circle0)
    allcfs.crazyflies[1].uploadTrajectory(1, 0, circle1)
    allcfs.crazyflies[2].uploadTrajectory(1, 0, circle2)

    params = {'ctrlFlip/T0': T0, 'ctrlFlip/T1': T1}

    for ID in allcfs.crazyfliesById:
        print("Here is crazyflie " + str(ID))
        flipInitParams(allcfs.crazyfliesById[ID])

    # start flying!
    initHeight = 0.4
    numDrones = len(allcfs.crazyflies)
    N = 10
    T = 63
    r = 0.4
    xinit = [r * np.cos(2 * np.pi * k / numDrones) for k in range(numDrones)]
    yinit = [r * np.sin(2 * np.pi * k / numDrones) for k in range(numDrones)]
    x = np.array([0, 0, 0])
    y = np.array([0.5, 0, -0.5])
    k = 0

    allcfs.takeoff(targetHeight=initHeight+0.2, duration=2)
    timeHelper.sleep(2)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([xinit[k], yinit[k], initHeight + 0.2]), 0, 2)
        k = k+1
    k = 0
    timeHelper.sleep(3)
    for cf in allcfs.crazyflies:
        # average motor PWMs
        cf.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)
    for cf in allcfs.crazyflies:
        # switch on PWM feedforward correction
        cf.setParam('motorPowerSet/isAv', 0)
        cf.setParam('motorPowerSet/isFF', 1)

    timeHelper.sleep(1)

    # goCircles(N, T, initHeight+0.2, r, numDrones)
    goCircles2()
    timeHelper.sleep(1)

    for cf in allcfs.crazyflies:
        cf.setParam('stabilizer/controller', 4)  # flip controller
        cf.goTo(np.array([x[k], y[k], initHeight + 0.2]), 0, 2)
        k = k+1
    k = 0
    timeHelper.sleep(3)

    for ID in allcfs.crazyfliesById:
        cf = allcfs.crazyfliesById[ID]
        # cf.takeoff(targetHeight=initHeight+0.2, duration=2)

        cf.setParam('locSrv/extQuatStdDev', 1.0)
        timeHelper.sleep(2)

        cf.setParam('ctrlFlip/isFlipControl', 1)
        timeHelper.sleep(T0 + T1)  # wait until the maneuver is over

        timeHelper.sleep(0.9)
        cf.goTo(np.array([x[k], y[k], 0.6]), 0, 2)
        cf.setParam('locSrv/extQuatStdDev', 0.05)

        # timeHelper.sleep(1)
        cf.setParam('ctrlFlip/wasFlipControl', 0)
        # cf.goTo(np.array([x[k], y[k], 0.6]), 0, 2)
        # timeHelper.sleep(2)
        # cf.land(targetHeight=0.06, duration=3)
        k = k+1
    k = 0
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([x[k], y[k], 0.6]), 0, 2)
        k = k+1
    # timeHelper.sleep(2)
    allcfs.land(targetHeight=0.06, duration=3)
