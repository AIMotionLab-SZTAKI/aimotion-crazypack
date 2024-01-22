#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm(crazyflies_yaml="../../launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj0 = uav_trajectory.Trajectory()
    traj0.loadcsv("../csv/figure8_yaw_v2.csv")

    TIMESCALE = 0.8
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj0)

    allcfs.takeoff(targetHeight=1.0, duration=2.0)
    timeHelper.sleep(2.5)
    for cf in allcfs.crazyflies:
        pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
        cf.goTo(pos, 0, 2.0)
    timeHelper.sleep(2.5)

    allcfs.startTrajectory(0, timescale=TIMESCALE)
    timeHelper.sleep(traj0.duration * TIMESCALE + 1.0)

    T0 = 0.0
    T1 = 0.7

    traj1 = uav_trajectory.Trajectory()

    traj1.loadcsv("../src/flip_traj_23.csv")


    params = {'ctrlFlip/T0': T0, 'ctrlFlip/T1': T1}

    # send the parameters
    cf = allcfs.crazyfliesById[3]
    cf.uploadTrajectory(0, 0, traj1)
    cf.setParams(params)
    cf.setParam('stabilizer/controller', 4)
    cf.setParam('ctrlFlip/controller_type', 2)
    cf.setParam('ctrlFlip/z0', 2.0)
    cf.setParam('ctrlFlip/zmin', 0.05)
    cf.setParam('ctrlFlip/rec_sp_z', 0.6)
    cf.setParam('ctrlGeom/mass', 0.036)
    cf.setParam('ctrlGeom/robust', 0)
    cf.setParam('ctrlGeom/delta_R', 0.002)
    cf.setParam('ctrlGeom/gp', 0)
    initHeight = 0.6
    x = 0.4
    y = 0
    init_pos = cf.position()
    cf.goTo(np.array([x, y, initHeight + 0.2]), 0, 2)

    timeHelper.sleep(3)

    # average motor PWMs
    cf.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)

    # switch on PWM feedforward correction
    cf.setParam('motorPowerSet/isAv', 0)
    cf.setParam('motorPowerSet/isFF', 1)
    timeHelper.sleep(5)

    ## Begin flip #############################
    # for cf in allcfs.crazyflies:
    cf.setParam('ctrlFlip/isFlipControl', 1)
    cf.startTrajectory(0, timescale=1)
    timeHelper.sleep(T0 + T1)
    for k in range(40):
        cf.cmdPosition(np.array([x, y, 0.8]), 0)
        timeHelper.sleepForRate(20)
    cf.notifySetpointsStop()
    cf.goTo(np.array([x, y, 0.8]), 0, 0.1)
    timeHelper.sleep(1)
    cf.setParam('ctrlFlip/wasFlipControl', 0)  # TODO
    timeHelper.sleep(2)
    cf.goTo(init_pos, 0, 2)
    timeHelper.sleep(2)

    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
