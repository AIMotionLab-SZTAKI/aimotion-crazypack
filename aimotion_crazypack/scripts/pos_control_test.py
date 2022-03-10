#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    circle = uav_trajectory.Trajectory()
    circle.loadcsv("csv/circle_traj_0.csv")

    # send the parameters
    for cf in allcfs.crazyflies:
        cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
        cf.setParam('stabilizer/controller', 2)
        # cf.setParam('ctrlGeom/mass', 0.0325)
        cf.uploadTrajectory(0, 0, circle)


    allcfs.takeoff(targetHeight=0.4, duration=2)
    timeHelper.sleep(3)
    for cf in allcfs.crazyflies:
        cf.goTo(np.array([0.4, 0, 0.4]), 0, 2)
    timeHelper.sleep(2)
    TIMESCALE = 0.5
    for cf in allcfs.crazyflies:
        cf.startTrajectory(0, timescale=TIMESCALE, reverse=False)
    timeHelper.sleep(circle.duration*TIMESCALE)
    timeHelper.sleep(1)
    allcfs.land(targetHeight=0.07, duration=2)
    timeHelper.sleep(2)
