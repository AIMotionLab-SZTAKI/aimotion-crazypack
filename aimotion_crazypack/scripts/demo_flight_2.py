#!/usr/bin/env python


import numpy as np
from pycrazyswarm import *

# Reading initial positions
import yaml
import os

cwd = os.getcwd()
cwd = cwd
with open(cwd + "/yaml/initialPosition.yaml", "r") as file:
    initialPosition = yaml.load(file, Loader=yaml.FullLoader)
# Setting targetHeight
targetHeight = initialPosition['crazyflies'][0]['initialPosition'][-1]
zoffset = 0.0
# Loading trajectories
import uav_trajectory

traj1 = uav_trajectory.Trajectory()
traj1.loadcsv(cwd + "/csv/vehicle01.csv")
traj2 = uav_trajectory.Trajectory()
traj2.loadcsv(cwd + "/csv/vehicle02.csv")
traj3 = uav_trajectory.Trajectory()
traj3.loadcsv(cwd + "/csv/vehicle03.csv")
trajectories = [traj1, traj2, traj3]
# traj4 = uav_trajectory.Trajectory()
# traj4.loadcsv(cwd + "/csv/vehicle3.csv")
# trajectories = [traj1, traj2, traj3, traj4]

if __name__ == "__main__":

    "Running crazyswarm"
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # timeHelper.sleep(8)
    # Take off
    for j, cf in enumerate(allcfs.crazyflies):
        cf.takeoff(targetHeight=targetHeight + j * zoffset - zoffset, duration=3)
    timeHelper.sleep(3.0)

    # Go to initial position
    for j, cf in enumerate(allcfs.crazyflies):
        pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
        print(pos)
        cf.goTo(pos + np.array([0, 0, j * zoffset - zoffset]), 0, 2.0)
        # cf.goTo(np.array([1, 0, targetHeight]), 0, 2.0)
        # cf.goTo(np.array(initialPosition['crazyflies'][0]['initialPosition']), 0, 2.0)
        print(pos)
    timeHelper.sleep(2)

    # Upload trajectories to cf
    for j, cf in enumerate(allcfs.crazyflies):
        cf.uploadTrajectory(0, 0, trajectories[0])

    # SAFETY CHECK
    print("Starting position okay? Press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    timeHelper.sleep(3)

    # Trajectory following
    reverse = True
    timescale = 1.0
    # allcfs.setParam('ctrlMel/log_data', 1)
    allcfs.startTrajectory(0, timescale=timescale, reverse=False, relative=True)
    timeHelper.sleep(traj1.duration * timescale + 1.0)
    # allcfs.crazyflies[0].goTo(np.array([-0.66, -0.27, targetHeight]), 0, 4.0)
    #


    # timeHelper.sleep(4)
    for j, cf in enumerate(allcfs.crazyflies):
        cf.cmdPosition(np.array([-0.66, -0.27, targetHeight]))
        cf.uploadTrajectory(1, 0, trajectories[1])
        cf.notifySetpointsStop(1)
        cf.startTrajectory(1, timescale=timescale, reverse=False, relative=True)
    timeHelper.sleep(traj2.duration * timescale + 0.0)

    # for j, cf in enumerate(allcfs.crazyflies):
    #     pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
    #     cf.cmdPosition(np.array(pos + np.array([0, 0, j * zoffset - zoffset])))
    #     cf.uploadTrajectory(2, 0, trajectories[2])
    #     cf.notifySetpointsStop(1)
    #     cf.startTrajectory(2, timescale=timescale, reverse=False, relative=True)
    # timeHelper.sleep(traj3.duration * timescale + 0.0)

    # allcfs.crazyflies[0].cmdStop()
    # for j, cf in enumerate(allcfs.crazyflies):
    #     cf.uploadTrajectory(2, 0, trajectories[2])
    # allcfs.startTrajectory(2, timescale=timescale, reverse=False)
    # timeHelper.sleep(traj3.duration * timescale + 0.0)

    # if reverse:
    #     allcfs.startTrajectory(0, timescale=timescale, reverse=reverse)
    #     timeHelper.sleep(traj1.duration * timescale + 0.0)
    # allcfs.setParam('ctrlMel/log_data', 0)
    # Landing
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)
