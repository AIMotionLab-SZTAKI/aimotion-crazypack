#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

Z = 1

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
        # cf.setParam('ctrlGeom/mass', 0.0325)
        # cf.uploadTrajectory(0, 0, circle)


    allcfs.takeoff(targetHeight=Z, duration=1.0+Z)
    timeHelper.sleep(4)
    allcfs.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)
    allcfs.setParam('motorPowerSet/isAv', 0)
    allcfs.setParam('motorPowerSet/isFF', 1)
    print("press button to continue... :)")
    TIMESCALE = 0.6
    swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.07, duration=1.0+Z)
    timeHelper.sleep(1.0+Z)
