#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *



# --- Peti declarations --- #
H = 1.4
U1, Th1, T1 = 0.5259, -42.346, 0.08219
U2, Th2, T2 = 0.3795, 297.346, 0.22265
U3, Th3, T3 = 0.17489, 0.0, 0.1209
U4, Th4, T4 = 0.3795, -297.346, 0.22193
U5, Th5, T5 = 0.50265, 59.2655, 0.05508
# Adjust flip parameters
U1 = 0.4
T1 = 0.2
Th1 = -10
T2 = np.array([0.165, 0.165, 0.175])
T3 = 0
Th2 = Th2 * 31000 / 24400
Th4 = -Th2
T4 = 0.2
U5 = U1
T5 = 0
Th5 = -Th1


def flipInitParams(cf, T1, T2, T3, T4):

    # Last working set: 0.5, 0.2, 0, 0.14, 0 ...
    #                   0.45, 0.3, -10, 0.16, 0 ... for cf5 (with better feedforward)
    #                   0.45, 0.2, -10, 0.177, 0 ... for cf3 (with better feedforward)
    #                   0.45, 0.3, -10, 0.166, 0 ... for cf2 (with better feedforward)
    # With starting momentum: 0.4, 0.2, -10, 0.18, 0 ... for cf5
    #                         0.4, 0.2, -10, 0.16, 0... for cf2
    #                         0.4, 0.2, -10, 0.175, 0... for cf3
    #                         0.4, 0.2, -10, 0.17, 0... for cf4

    # send the parameters
    cf.setParam('motorPowerSet/enable', 0)  # we do not want to control the motors manually
    cf.setParam('stabilizer/controller', 4)  # PID controller
    params = {'ctrlFlip/U1': U1, 'ctrlFlip/Th1': Th1, 'ctrlFlip/T1': T1,
              'ctrlFlip/U2': U2, 'ctrlFlip/Th2': Th2, 'ctrlFlip/T2': float(T2),
              'ctrlFlip/U3': U3, 'ctrlFlip/Th3': Th3, 'ctrlFlip/T3': T3,
              'ctrlFlip/U4': U4, 'ctrlFlip/Th4': Th4, 'ctrlFlip/T4': T4,
              'ctrlFlip/U5': U5, 'ctrlFlip/Th5': Th5, 'ctrlFlip/T5': T5}
    cf.setParams(params)


def executeFlip(cf, T1, T2, T3, T4):

    position = cf.position()

    # average motor PWMs
    cf.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)

    # switch on PWM correction
    cf.setParam('motorPowerSet/isAv', 0)
    cf.setParam('motorPowerSet/isFF', 1)
    timeHelper.sleep(2)

    # start to lift
    cf.goTo(np.array([position[0], position[1], initHeight + 2]), 0, 2)
    timeHelper.sleep(1.2)

    ## Begin flip #############################
    cf.setParam('ctrlFlip/isFlipControl', 1)
    timeHelper.sleep(T1 + T2 + T3 + T4)  # wait until the maneuver is over
    cf.goTo(np.array([position[0], position[1], 0.6]), 0, 1)

    # recover
    timeHelper.sleep(1)
    cf.cmdPosition(np.array([position[0], position[1], 0.6]), 0)
    cf.setParam('ctrlFlip/wasFlipControl', 0)

    # really get back to initial position
    for k in range(40):
        cf.cmdPosition(np.array([position[0], position[1], 0.6]), 0)
        timeHelper.sleepForRate(20)


def goCircles(N, T, numDrones):
    for k in range(numDrones):
        allcfs.crazyflies[k].goTo(
            np.array([r * np.cos(2 * np.pi * k / numDrones), r * np.sin(2 * np.pi * k / numDrones), h]), 0, 3)
    timeHelper.sleep(3)
    for t in range(T):
        for k in range(numDrones):
            allcfs.crazyflies[k].goTo(np.array(
                [r * np.cos(t / N + 2 * np.pi * k / numDrones), r * np.sin(t / N + 2 * np.pi * k / numDrones), h + t/T*2/3]), 0,
                                      0.1)
        timeHelper.sleepForRate(10)
    timeHelper.sleep(1)
    for t in range(T):
        for k in range(numDrones):
            allcfs.crazyflies[k].goTo(np.array(
                [r * np.cos(-t / N + 2 * np.pi * k / numDrones), r * np.sin(-t / N + 2 * np.pi * k / numDrones), h + 2/3 - t/T*2/3]), 0,
                                      0.1)
        timeHelper.sleepForRate(10)



if __name__ == "__main__":
    # initialize crazyflie
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    # Initialize flip parameters
    for i, cf in enumerate(allcfs.crazyflies):
        flipInitParams(cf, T1, T2[i], T3, T4)
    # start flying!
    h = 0.25
    r = 0.5
    allcfs.takeoff(targetHeight=h, duration=3)
    timeHelper.sleep(3)
    numDrones = len(allcfs.crazyflies)
    N = 5
    T = 63
    goCircles(N, T, numDrones)
    initHeight = 0.4
    FLIP_X = 0.0
    '''
    print("Can we do the flip already? :D :P Press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    timeHelper.sleep(1)
    numDrone = 0
    # allcfs.crazyflies[numDrone].takeoff(targetHeight=initHeight + 0.2, duration=3)
    # timeHelper.sleep(3)
    # allcfs.crazyflies[1].land(targetHeight=0.06, duration=2)  # goTo(np.array([-0.6, 0.6, initHeight]), 0, 3)
    # allcfs.crazyflies[2].land(targetHeight=0.06, duration=2)  # goTo(np.array([-0.6, -0.6, initHeight]), 0, 3)
    # allcfs.crazyflies[numDrone].goTo(np.array([0.1, 0, initHeight + 0.2]), 0, 3)
    # timeHelper.sleep(4)
    # executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)
    # allcfs.crazyflies[numDrone].goTo(np.array([FLIP_X, 0.6, 0.6]), 0, 3)
    # timeHelper.sleep(3)
    # allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)
    # timeHelper.sleep(3)
    # allcfs.land(targetHeight=0.06, duration=2)

    goCircles(N, T, numDrones)



    # SAFETY CHECK
    print("Can we do the flip already? :D :P Press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    timeHelper.sleep(1)
    
    numDrone = 1
    # allcfs.crazyflies[numDrone].takeoff(targetHeight=initHeight + 0.2, duration=3)
    timeHelper.sleep(2)
    allcfs.crazyflies[0].land(targetHeight=0.06, duration=2)  # .goTo(np.array([0.6, 0.6, initHeight-0.15]), 0, 3)
    allcfs.crazyflies[2].land(targetHeight=0.06, duration=2)  # .goTo(np.array([-0.6, -0.6, initHeight-0.15]), 0, 3)
    allcfs.crazyflies[numDrone].goTo(np.array([-0.4, -0.2, initHeight + 0.2]), 0, 3)
    timeHelper.sleep(4)
    executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)

    # goCircles(N, T, numDrones)
    #
    # timeHelper.sleep(2)
    # allcfs.crazyflies[0].land(targetHeight=0.06, duration=2)  # .goTo(np.array([0.6, 0.6, initHeight-0.15]), 0, 3)
    # allcfs.crazyflies[2].land(targetHeight=0.06, duration=2)  # .goTo(np.array([-0.6, -0.6, initHeight-0.15]), 0, 3)
    # allcfs.crazyflies[numDrone].goTo(np.array([FLIP_X, 0, initHeight + 0.2]), 0, 3)
    # timeHelper.sleep(4)
    # executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)
    allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)
    '''
    # allcfs.crazyflies[numDrone].goTo(np.array([FLIP_X, -0.6, 0.6]), 0, 3)
    # timeHelper.sleep(3)
    # allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)

    # allcfs.land(targetHeight=0.06, duration=3)
    # goCircles(N, T, numDrones)
    #
    # # SAFETY CHECK
    print("Can we do the flip already? :D :P Press button to continue...")
    # swarm.input.waitUntilButtonPressed()
    timeHelper.sleep(1)

    #
    numDrone = 2
    # allcfs.crazyflies[numDrone].takeoff(targetHeight=initHeight + 0.2, duration=3)
    # timeHelper.sleep(3)
    allcfs.crazyflies[0].land(targetHeight=0.06, duration=2)  # .goTo(np.array([0.6, 0.6, initHeight-0.15]), 0, 3)
    allcfs.crazyflies[1].land(targetHeight=0.06, duration=2)  # .goTo(np.array([-0.6, 0.6, initHeight-0.15]), 0, 3)
    allcfs.crazyflies[numDrone].goTo(np.array([FLIP_X, 0, initHeight + 0.2]), 0, 3)
    timeHelper.sleep(4)
    executeFlip(allcfs.crazyflies[numDrone], T1, T2[numDrone], T3, T4)
    # allcfs.crazyflies[numDrone].goTo(np.array([FLIP_X, 0.0, 0.6]), 0, 3)
    # timeHelper.sleep(3)
    allcfs.crazyflies[numDrone].land(targetHeight=0.06, duration=3)

    # land

    # allcfs.land(targetHeight=0.06, duration=3)