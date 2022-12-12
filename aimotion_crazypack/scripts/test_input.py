#!/usr/bin/env python
import time

import numpy as np

Z = 0.4

if __name__ == "__main__":

    while True:
        string_coords = input("x;y;z + enter\n")
        try:
            new_coords = np.array([float(s) for s in string_coords.split(";")])
            print("Your new coordinates are\n")
            print(new_coords)
        except:
            print("Invalid input, landing......\n")
            break
        time.sleep(2)


    print("Landing")
    # print("press button to continue... :)")
    # TIMESCALE = 0.6
    # swarm.input.waitUntilButtonPressed()

    # allcfs.land(targetHeight=0.07, duration=3.0+Z)
    # timeHelper.sleep(3.0+Z)
