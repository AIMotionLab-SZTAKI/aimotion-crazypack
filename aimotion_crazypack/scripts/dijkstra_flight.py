#!/usr/bin/env python


import numpy as np
from pycrazyswarm import *
import argparse

# Reading initial positions
import yaml
import os

cwd = os.getcwd()
cwd = cwd
with open(cwd + "/dijkstra/yaml/initialPosition.yaml", "r") as file:
    initialPosition = yaml.load(file, Loader=yaml.FullLoader)
# Setting targetHeight
targetHeight = initialPosition['crazyflies'][0]['initialPosition'][-1]
zoffset = 0.0 # 0.1
# Loading trajectories
import uav_trajectory

traj1 = uav_trajectory.Trajectory()
traj1.loadcsv(cwd + "/dijkstra/csv/vehicle0.csv")
traj2 = uav_trajectory.Trajectory()
traj2.loadcsv(cwd + "/dijkstra/csv/vehicle1.csv")
trajectories = [traj1, traj2]
# trajectories = [traj1]
# traj4 = uav_trajectory.Trajectory()
# traj4.loadcsv(cwd + "/csv/vehicle3.csv")
# trajectories = [traj1, traj2, traj3, traj4]

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--takeoff', default=False, type=bool)
    parser.add_argument('--land', default=False, type=bool)
    ARGS = parser.parse_args()

    "Running crazyswarm"
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    for cf in allcfs.crazyflies:
        cf.setParam('stabilizer/controller', 1)

    # timeHelper.sleep(8)
    # Take off
    if ARGS.takeoff == True:
        for j, cf in enumerate(allcfs.crazyflies):
            cf.takeoff(targetHeight=targetHeight + j * zoffset - zoffset, duration=3)
        timeHelper.sleep(3.0)
    else:
        pass

    # Go to initial position
    for j, cf in enumerate(allcfs.crazyflies):
        pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
        # print(pos)
        cf.goTo(pos + np.array([0, 0, j * zoffset - zoffset]), 0, 2.0)
        # cf.goTo(np.array([1, 0, targetHeight]), 0, 2.0)
        # cf.goTo(np.array(initialPosition['crazyflies'][0]['initialPosition']), 0, 2.0)
        # print(pos)
    timeHelper.sleep(1)

    # Upload trajectories to cf
    for j, cf in enumerate(allcfs.crazyflies):
        cf.uploadTrajectory(0, 0, trajectories[j])


    print("----------------------")
    print("----------------------")
    print("")
    print("Trajectory uploaded")
    print("")
    print("----------------------")
    print("----------------------")
    # SAFETY CHECK
    # print("Starting position okay? Press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    # timeHelper.sleep(3.0)

    # Trajectory following
    reverse = False
    timescale = 0.75
    # allcfs.setParam('ctrlMel/log_data', 1)
    allcfs.startTrajectory(0, timescale=timescale, reverse=False)
    # timeHelper.sleep(traj1.duration * timescale + 0.0)
    if reverse:
        allcfs.startTrajectory(0, timescale=timescale, reverse=reverse)
        # timeHelper.sleep(traj1.duration * timescale + 0.0)


    from path_planner import *
    run_main(select=False)

    if ARGS.land == True:
        # allcfs.setParam('ctrlMel/log_data', 0)
        # Landing
        allcfs.land(targetHeight=0.06, duration=2.0)
        timeHelper.sleep(2.0)
    else:
        pass
