#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # circle = uav_trajectory.Trajectory()
    # circle.loadcsv("csv/circle_traj_3.csv")

    # send the parameters
    for cf in allcfs.crazyflies:
        cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
        cf.setParam('stabilizer/controller', 2)
        # cf.setParam('usd/logging', 1)
        # cf.setParam('ctrlGeom/mass', 0.0325)
        # cf.uploadTrajectory(0, 0, circle)
    Z = [0.2, 0.5, 0.6, 0.2, 0.5, 0.45, 0.65]

    for i, cf in enumerate(allcfs.crazyflies):
        cf.takeoff(targetHeight=Z[i], duration=2)
    timeHelper.sleep(2)
    # allcfs.crazyflies[0].goTo(np.array([0.3, 0, 1.3]), 0, 3)
    # allcfs.setParam('motorPowerSet/isAv', 1)
    # timeHelper.sleep(3)
    # allcfs.setParam('motorPowerSet/isAv', 0)
    # allcfs.setParam('motorPowerSet/isFF', 1)
    print("press button to continue... :)")
    TIMESCALE = 0.6
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.07, duration=3.0)
    timeHelper.sleep(3.0)
    # allcfs.setParam('usd/logging', 0)
