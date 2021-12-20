#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

def goCircles2():
    TIMESCALE = 0.7
    for cf in allcfs.crazyflies[0:4]:
        cf.startTrajectory(1, timescale=TIMESCALE, reverse=False)

    allcfs.crazyflies[4].setParam('ring/solidRed', 255)
    allcfs.crazyflies[4].setParam('ring/solidGreen', 160)
    allcfs.crazyflies[4].setParam('ring/solidBlue', 0)

    n_of_blinks = 10
    blink_length = circle0.duration*TIMESCALE / n_of_blinks
    for i in range(n_of_blinks):
        if i%2:
            for cf in allcfs.crazyflies[0:4]:
                cf.setParam('ring/solidRed', 0)
                cf.setParam('ring/solidGreen', 255)
                cf.setParam('ring/solidBlue', 0)
        else:
            for cf in allcfs.crazyflies[0:4]:
                cf.setParam('ring/solidRed', 255)
                cf.setParam('ring/solidGreen', 0)
                cf.setParam('ring/solidBlue', 0)
        allcfs.crazyflies[4].goTo(np.array([-0.15, 0.07, 1.9]), 0, 0.1)
        timeHelper.sleep(blink_length)


    # cf.setParam('motorPowerSet/isAv', 0)
    # timeHelper.sleepForRate(10)

    # timeHelper.sleep(circle0.duration*TIMESCALE)

if __name__ == "__main__":

    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    circle0 = uav_trajectory.Trajectory()
    circle0.loadcsv("csv/circ_traj_0.csv")
    circle1 = uav_trajectory.Trajectory()
    circle1.loadcsv("csv/circ_traj_1.csv")
    circle2 = uav_trajectory.Trajectory()
    circle2.loadcsv("csv/circ_traj_2.csv")
    circle3 = uav_trajectory.Trajectory()
    circle3.loadcsv("csv/circ_traj_3.csv")

    allcfs.crazyflies[0].uploadTrajectory(1, 0, circle0)
    allcfs.crazyflies[1].uploadTrajectory(1, 0, circle1)
    allcfs.crazyflies[2].uploadTrajectory(1, 0, circle2)
    allcfs.crazyflies[3].uploadTrajectory(1, 0, circle3)

    for cf in allcfs.crazyflies:
        cf.setParam('ring/effect', 7)
        cf.setParam('ring/solidRed', 0)
        cf.setParam('ring/solidGreen', 0)
        cf.setParam('ring/solidBlue', 0)

    for ID in allcfs.crazyfliesById:
        print("Here is crazyflie " + str(ID))

    # start flying!
    initHeight = 0.8
    numDrones = 5
    N = 10
    T = 63
    r = [0.85, 0.7, 0.55, 0.4, 0]
    h = [0.6, 0.9, 1.2, 1.6, 1.9]
    xinit = [r[k] * np.cos(2 * np.pi * k / 4) for k in range(4)]
    yinit = [r[k] * np.sin(2 * np.pi * k / 4) for k in range(4)]
    xinit.append(-0.15)
    yinit.append(0.07)

    xfinal = [r[0] * np.cos(2 * np.pi * k / 4) for k in range(4)]
    yfinal = [r[0] * np.sin(2 * np.pi * k / 4) for k in range(4)]
    xfinal.append(r[0]/1.41)
    yfinal.append(-r[0]/1.41)
    # x = np.array([0, 0, 0])
    # y = np.array([0.5, 0, -0.5])
    k = 0

    for i in range(numDrones):
        allcfs.crazyflies[i].takeoff(targetHeight=h[i], duration=3)
    timeHelper.sleep(3)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([xinit[k], yinit[k], h[k]]), 0, 2)
        k = k+1
    k = 0
    timeHelper.sleep(6)

    goCircles2()
    timeHelper.sleep(1)

    # for cf in allcfs.crazyflies:
    #     cf.goTo(np.array([xfinal[k], yfinal[k], h[k]]), 0, 1)
    #     k = k+1
    allcfs.crazyflies[0].goTo(np.array([0.7, 0, 1.8]), 0, 3)
    timeHelper.sleep(3)
    allcfs.crazyflies[0].goTo(np.array([-0.15, 0.07, 1.9]), 0, 3)
    allcfs.crazyflies[1].goTo(np.array([-0.8, 0.9, 1]), 0, 3)
    allcfs.crazyflies[2].goTo(np.array([-0.8, 0.2, 1.2]), 0, 3)
    allcfs.crazyflies[3].goTo(np.array([-0.8, -0.9, 1.6]), 0, 3)
    allcfs.crazyflies[4].goTo(np.array([-0.8, -0.2, 1.7]), 0, 3)

    # Turn Crazyflie 5 to green
    allcfs.crazyflies[4].setParam('ring/solidRed', 0)
    allcfs.crazyflies[4].setParam('ring/solidGreen', 255)
    allcfs.crazyflies[4].setParam('ring/solidBlue', 0)

    timeHelper.sleep(5)

    allcfs.crazyflies[1].goTo(np.array([-0.8, 0.6, 0.7]), 0, 3)
    allcfs.crazyflies[2].goTo(np.array([-1, 0.2, 0.7]), 0, 3)
    allcfs.crazyflies[3].goTo(np.array([-0.8, -0.6, 0.7]), 0, 3)
    allcfs.crazyflies[4].goTo(np.array([-1, -0.2, 0.7]), 0, 3)

    timeHelper.sleep(3)

    for cf in allcfs.crazyflies:
        cf.setParam('ring/solidRed', 255)
        cf.setParam('ring/solidGreen', 160)
        cf.setParam('ring/solidBlue', 0)

    timeHelper.sleep(12)
    allcfs.crazyflies[0].goTo(np.array([0.85, 0, 1.9]), 0, 3)
    timeHelper.sleep(3)
    allcfs.land(targetHeight=0.06, duration=3)
