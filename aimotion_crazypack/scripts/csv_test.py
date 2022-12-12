#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("csv/hook_up.csv")
    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("csv/hook_up_2.csv")
    # traj1.loadcsv("flip_traj.csv")

    TRIALS = 1
    TIMESCALE = 2
    for i in range(TRIALS):
        for cf in allcfs.crazyflies:
            cf.setParam('stabilizer/controller', 1)
            cf.uploadTrajectory(0, 0, traj1)

        allcfs.takeoff(targetHeight=1.0, duration=2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            cf.goTo(np.array([-0.5, 1, 1.4]), np.pi/2, 2.0)
        timeHelper.sleep(2.5)

        allcfs.startTrajectory(0, timescale=TIMESCALE, relative=False)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        for cf in allcfs.crazyflies:
            cf.uploadTrajectory(1, 0, traj2)
        allcfs.startTrajectory(1, timescale=TIMESCALE, relative=False)
        timeHelper.sleep(traj2.duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(3.0)
