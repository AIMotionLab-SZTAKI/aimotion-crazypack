#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

H = 1.4

if __name__ == "__main__":

    # Adjust flip parameters (these are empirical)
    ### Revert ###
    T0 = 1.2
    T1 = 0.7
    ###
    # T0 = 0.7
    # T1 = 0.7

    # initialize crazyflie
    swarm = Crazyswarm(crazyflies_yaml="../../launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = uav_trajectory.Trajectory()
    ### Revert ###
    traj1.loadcsv("../src/flip_traj_lift.csv")
    ###
    # traj1.loadcsv("flip_traj.csv")

    params = {'ctrlFlip/T0': T0, 'ctrlFlip/T1': T1}

    # send the parameters
    # for cf in allcfs.crazyflies:
    cf = allcfs.crazyfliesById[10]
    ### Revert ###
    cf.uploadTrajectory(0, 0, traj1)
    ###
    cf.setParams(params)
    cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
    cf.setParam('stabilizer/controller', 4)
    ### Revert ###
    # cf.setParam('ctrlFlip/controller_type', 2)
    ###
    cf.setParam('ctrlFlip/controller_type', 2)
    cf.setParam('ctrlFlip/z0', 2.0)
    cf.setParam('ctrlFlip/zmin', 0.05)
    ### Revert ###
    cf.setParam('ctrlFlip/rec_sp_z', 0.6)
    ###
    # cf.setParam('ctrlFlip/rec_sp_z', 0.7)
    # cf.setParam('ctrlGeom/mass', 0.0325)
    ### Revert ###
    cf.setParam('ctrlMel/mass', 0.0325)
    # cf.setParam('ctrlGeom/gp', 1)
    ###

    # timeHelper.sleep(6)

    # start flying!
    initHeight = 0.4
    x = 0
    y = 0
    cf.takeoff(targetHeight=initHeight+0.2, duration=2)
    timeHelper.sleep(2)
    init_pos = cf.position()
    cf.goTo(np.array([-0.8, y, 0.8]), 0, 2)
    timeHelper.sleep(2)
    cf.goTo(np.array([x, y, initHeight+0.2]), 0, 2)
    # allcfs.crazyflies[0].setParam('stabilizer/controller', 4)  # flip controller

    timeHelper.sleep(3)

    # average motor PWMs
    cf.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)

    # switch on PWM feedforward correction
    cf.setParam('motorPowerSet/isAv', 0)
    cf.setParam('motorPowerSet/isFF', 1)
    timeHelper.sleep(2)

    ## Begin flip #############################
    # for cf in allcfs.crazyflies:
    cf.setParam('ctrlFlip/isFlipControl', 1)
        ### Revert ###
    cf.startTrajectory(0, timescale=1)
        # cf.setParam('usd/logging', 1)
        ###
    # allcfs.setParam('locSrv/extQuatStdDev', 1.0)
    # allcfs.crazyflies[0].goTo(np.array([x, y, 0.9]), 0, 0.01)
    timeHelper.sleep(T0 + T1)
    # allcfs.crazyflies[0].setParam('ctrlFlip/controller_type', 1)

    # timeHelper.sleep(0.9)

    # allcfs.crazyflies[0].setParam('locSrv/extQuatStdDev', 0.05)
    # x_cur = allcfs.crazyflies[0].position()[0]
    # print(x_cur)
    # for cf in allcfs.crazyflies:
    for k in range(40):
        cf.cmdPosition(np.array([0, y, 0.8]), 0)
        timeHelper.sleepForRate(20)
    cf.notifySetpointsStop()
    cf.goTo(np.array([0, y, 0.8]), 0, 0.1)
    timeHelper.sleep(1)
    # for cf in allcfs.crazyflies:
    cf.setParam('ctrlFlip/wasFlipControl', 0)    # TODO
        # cf.setParam('locSrv/extQuatStdDev', 0.05)
    timeHelper.sleep(2)
    cf.goTo(np.array([-0.8, y, 0.8]), 0, 2)
    timeHelper.sleep(2)
    cf.goTo(init_pos, 0, 2)
    timeHelper.sleep(2)
    ### Revert ###
    # for cf in allcfs.crazyflies:
    #     for k in range(40):
    #         cf.cmdPosition(np.array([0, y, 0.6]), 0)
    #         timeHelper.sleepForRate(20)
    #     cf.notifySetpointsStop()
    #     cf.setParam('usd/logging', 0)

    # allcfs.crazyflies[0].goTo(np.array([x, y, 0.6]), 0, 2)

    # timeHelper.sleep(1)
    # for cf in allcfs.crazyflies:
    #     cf.setParam('ctrlFlip/wasFlipControl', 0)    # TODO
    #     cf.setParam('locSrv/extQuatStdDev', 0.05)
    # timeHelper.sleep(2)
    ###
    # land
    cf.land(targetHeight=0.06, duration=3)
