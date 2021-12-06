#!/usr/bin/env python

import numpy as np

from pycrazyswarm import *
import uav_trajectory

import main as main



if __name__ == "__main__":
    
    "Running the optimization algorithm"
    
    	
    # for i in range(max_trial_length):
        
    
    "Running crazyswarm"
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # Trajectories
    traj1 = uav_trajectory.Trajectory()
    traj1.loadcsv("vehicle1.csv")
    traj2 = uav_trajectory.Trajectory()
    traj2.loadcsv("vehicle2.csv")
    traj3 = uav_trajectory.Trajectory()
    traj3.loadcsv("vehicle3.csv")

    trajectories = [traj1, traj2, traj3]

    TRIALS = 1
    TIMESCALE = 1.0
    for i in range(TRIALS):
        for j, cf in enumerate(allcfs.crazyflies):
            cf.uploadTrajectory(0, 0, trajectories[j])

        allcfs.takeoff(targetHeight = 1.0, duration = 2.0)
        timeHelper.sleep(2.5)
        for cf in allcfs.crazyflies:
            pos = np.array(cf.initialPosition) + np.array([0, 0, 1.0])
            cf.goTo(pos, 0, 2.0)
        timeHelper.sleep(2.5)

        allcfs.startTrajectory(0, timescale = TIMESCALE)
        timeHelper.sleep(traj1.duration * TIMESCALE + 2.0)
        allcfs.startTrajectory(0, timescale = TIMESCALE, reverse = True)
        timeHelper.sleep(traj.duration * TIMESCALE + 2.0)

        allcfs.land(targetHeight = 0.06, duration = 2.0)
        timeHelper.sleep(3.0)

        
