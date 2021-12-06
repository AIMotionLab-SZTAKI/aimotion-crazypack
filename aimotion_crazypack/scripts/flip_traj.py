#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("flip_traj.csv")

    TIMESCALE = 1
    for cf in allcfs.crazyflies:
        cf.uploadTrajectory(0, 0, traj1)
        allcfs.takeoff(targetHeight=1.8, duration=2.0)
        timeHelper.sleep(2)
        cf.setParam('stabilizer/controller', 5)
        cf.setParam('ctrlGeom/timescale', TIMESCALE)

        pos = np.array([0, 0, 1.8])
        cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2)
        # average motor PWMs
        allcfs.setParam('motorPowerSet/isAv', 1)
        timeHelper.sleep(3)

        # switch on PWM feedforward correction
        allcfs.setParam('motorPowerSet/isAv', 0)
        allcfs.setParam('motorPowerSet/isFF', 1)
        timeHelper.sleep(2)
        cf.setParam('locSrv/extQuatStdDev', 1.0)
        # cf.setParam('ctrlGeom/kd_omega_rp', 0.000005)

    # kRs = np.linspace(0.0095, 0.015, 2)
    # kwmuls = np.linspace(0.3, 0.4, 2)
    # for kR in kRs:
    #     for kwmul in kwmuls:

    for cf in allcfs.crazyflies:
        # cf.setParam('ctrlGeom/kR_xy', float(kR))
        # cf.setParam('ctrlGeom/kw_xy', float(kR*kwmul))  # 0.0027
        cf.startTrajectory(0, timescale=TIMESCALE)
        cf.setParam('ctrlGeom/mode', 1)
        timeHelper.sleep(traj1.duration * TIMESCALE)
        # cf.setParam('ctrlGeom/kR_xy', 0.0095)
        # cf.setParam('ctrlGeom/kw_xy', 0.0027)
        # cf.goTo(pos, 0, 2.5)
        timeHelper.sleep(0.5)
        cf.setParam('stabilizer/stop', 1)
    #
    # allcfs.land(targetHeight=0.04, duration=2.0)
    # timeHelper.sleep(3.0)
