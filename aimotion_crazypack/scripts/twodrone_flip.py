#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory
import subprocess

if __name__ == "__main__":

    # Adjust flip parameters (these are empirical)
    T0 = 1.5
    T1 = 0.7


    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("src/flip_traj_lift.csv")

    params = {'ctrlFlip/T0': T0, 'ctrlFlip/T1': T1}

    # send the parameters
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)
        cf.setParams(params)
        cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
        cf.setParam('stabilizer/controller', 4)
        cf.setParam('ctrlFlip/controller_type', 2)
        cf.setParam('ctrlFlip/z0', 2.0)
        cf.setParam('ctrlFlip/rec_sp_z', 0.6)
        cf.setParam('ctrlGeom/mass', 0.0325)
        cf.setParam('ctrlMel/mass', 0.0325)
        cf.setParam('ctrlGeom/gp', 0)
    # timeHelper.sleep(6)

    # start flying!
    initHeight = 0.4
    x = 0.3
    y = 0
    allcfs.takeoff(targetHeight=initHeight+0.2, duration=2)
    timeHelper.sleep(2)

    allcfs.crazyflies[0].goTo(np.array([x, y, initHeight+0.2]), 0, 2)
    allcfs.crazyflies[0].setParam('stabilizer/controller', 4)  # flip controller

    timeHelper.sleep(3)

    # average motor PWMs
    allcfs.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)

    # switch on PWM feedforward correction
    allcfs.setParam('motorPowerSet/isAv', 0)
    allcfs.setParam('motorPowerSet/isFF', 1)

    cf1 = allcfs.crazyflies[0]
    cf1.setParam('locSrv/extQuatStdDev', 1.0)
    timeHelper.sleep(2)

    ## Begin flip #############################
    cf1.setParam('ctrlFlip/isFlipControl', 1)
    cf1.startTrajectory(0, timescale=1)

    timeHelper.sleep(T0+T1)  # wait until the maneuver is over

    cf1.setParam('locSrv/extQuatStdDev', 0.05)
    for k in range(40):
        cf1.cmdPosition(np.array([0, y, 0.6]), 0)
        timeHelper.sleepForRate(20)
    cf1.notifySetpointsStop()
    cf1.goTo(np.array([-0.7, y, 0.6]), 0, 2)

    cf2 = allcfs.crazyflies[1]
    cf2.goTo(np.array([x, y, initHeight+0.2]), 0, 2)
    cf2.setParam('stabilizer/controller', 4)  # flip controller
    cf2.setParam('locSrv/extQuatStdDev', 1.0)
    timeHelper.sleep(2)

    cf2.setParam('ctrlFlip/isFlipControl', 1)
    cf2.startTrajectory(0, timescale=1)

    timeHelper.sleep(T0 + T1)  # wait until the maneuver is over

    cf2.setParam('locSrv/extQuatStdDev', 0.05)
    for k in range(40):
        cf2.cmdPosition(np.array([0, y, 0.6]), 0)
        timeHelper.sleepForRate(20)
    cf2.notifySetpointsStop()
    cf2.goTo(np.array([0.3, y, 0.6]), 0, 2)

    timeHelper.sleep(2)

    allcfs.land(targetHeight=0.06, duration=3)
