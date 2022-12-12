#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory

Z = 0.546

if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # circle = uav_trajectory.Trajectory()
    # circle.loadcsv("csv/circle_traj_3.csv")

    # send the parameters
    for cf in allcfs.crazyflies:
        cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
        cf.setParam('stabilizer/controller', 1)

    init_coords = np.array([0, 0, 0.5])
    prev_coords = init_coords
    new_coords = np.zeros(3)

    allcfs.takeoff(targetHeight=0.4, duration=0.4+Z)
    timeHelper.sleep(4)
    while True:
        string_coords = input("Give the coordinates x;y;z and press enter\n")
        try:
            new_coords = np.array([float(s) for s in string_coords.split(";")])
            print("Your new coordinates are\n")
            print(new_coords)
        except:
            print("Invalid input, landing......\n")
            break

        if abs(new_coords[0]) > 2.5 or abs(new_coords[1]) > 2.5 or new_coords[2] < 0 or new_coords[2] > 1.8:
            print("Coordinates are out of the flying space\n")
            continue
        dist = np.linalg.norm(new_coords-prev_coords)
        duration = dist*3
        for cf in allcfs.crazyflies:
            cf.goTo(new_coords, 0, duration)

        prev_coords = new_coords
        timeHelper.sleep(duration)

    print("Landing")
    # print("press button to continue... :)")
    # TIMESCALE = 0.6
    # swarm.input.waitUntilButtonPressed()

    allcfs.land(targetHeight=0.07, duration=3.0+Z)
    timeHelper.sleep(3.0+Z)
