#!/usr/bin/env python

import numpy as np
from pycrazyswarm import *
import uav_trajectory


def compute_controls(params):
    U1, t1, t3, U5, t5 = params
    Ixx = 1.4e-5
    length = 0.046
    mass = 0.028
    Umax = 0.5 / mass
    Umin = 0.1 / mass
    max_ang_vel = 23
    
    alpha = 2 * Ixx / mass / length / length
    p1 = -(Umax - U1) / alpha / length
    p2 = (Umax - Umin) / alpha / length / 2
    p3 = 0
    p4 = -1 * p2
    U2 = (Umax + Umin) / 2
    t2 = (max_ang_vel - p1 * t1) / p2
    U4 = U2
    U3 = Umin
    p5 = (Umax - U5) / alpha / length
    t4 = -(max_ang_vel + p5 * t5) / p4

    return t1, t2, t3, t4, t5, p1, p2, p3, p4, p5, U1, U2, U3, U4, U5


if __name__ == "__main__":

    # Adjust flip parameters 
    # params = [17.73, 0.22, 0.344, 17.7, 0.22]
    params = [17.8, 0.15, 0.2, 17.8, 0.12]
    T1, T2, T3, T4, T5, Th1, Th2, Th3, Th4, Th5, U1, U2, U3, U4, U5 = compute_controls(params)
    print((T1, T2, T3, T4, T5, Th1, Th2, Th3, Th4, Th5, U1, U2, U3, U4, U5))

    # initialize crazyflie
    swarm = Crazyswarm(crazyflies_yaml="../../launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # send the parameters
    # for cf in allcfs.crazyflies:
    cf = allcfs.crazyfliesById[3]

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
    cf.setParam('stabilizer/controller', 6)  # Flip FF
    cf.setParam('ctrlFlipFF/mass', 0.03)  # Flip FF
    cf.setParam('ctrlFlipFF/Ixx', 1.2e-5)  # 1.58e.5
    # cf.setParam('ctrlFlipFF/controller_type', 2)
    params = {'ctrlFlipFF/U1': U1, 'ctrlFlipFF/Th1': Th1, 'ctrlFlipFF/T1': T1,
              'ctrlFlipFF/U2': U2, 'ctrlFlipFF/Th2': Th2, 'ctrlFlipFF/T2': T2,
              'ctrlFlipFF/U3': U3, 'ctrlFlipFF/Th3': Th3, 'ctrlFlipFF/T3': T3,
              'ctrlFlipFF/U4': U4, 'ctrlFlipFF/Th4': Th4, 'ctrlFlipFF/T4': T4,
              'ctrlFlipFF/U5': U5, 'ctrlFlipFF/Th5': Th5, 'ctrlFlipFF/T5': 0*T5}
    cf.setParams(params)

    # start flying!
    initHeight = 0.8
    x = 0.4
    y = 0
    cf.takeoff(targetHeight=initHeight, duration=2)
    timeHelper.sleep(2)
    init_pos = cf.position()
    # cf.goTo(np.array([-0.8, y, 0.8]), 0, 2)
    # timeHelper.sleep(2)
    cf.goTo(np.array([x, y, initHeight]), 0, 2)
    # allcfs.crazyflies[0].setParam('stabilizer/controller', 4)  # flip controller

    timeHelper.sleep(3)

    # average motor PWMs
    cf.setParam('motorPowerSet/isAv', 1)
    timeHelper.sleep(3)

    # switch on PWM feedforward correction
    cf.setParam('motorPowerSet/isAv', 0)
    cf.setParam('motorPowerSet/isFF', 1)
    timeHelper.sleep(5)

    ## Begin flip #############################
    cf.setParam('ctrlFlipFF/isFlipControl', 1)
    cf.setParam('usd/logging', 1)
    timeHelper.sleep(T1 + T2 + T3 + T4 + T5 + 1)

    # for k in range(40):
    #     cf.cmdPosition(np.array([x, y, 0.8]), 0)
    #     timeHelper.sleepForRate(20)
    cf.setParam('usd/logging', 0)

    timeHelper.sleep(3)
    # cf.notifySetpointsStop()
    # cf.goTo(np.array([x, y, 0.8]), 0, 0.1)0
    # timeHelper.sleep(1)
    # cf.setParam('ctrlFlip/wasFlipControl', 0)    # TODO
    # timeHelper.sleep(2)
    # cf.goTo(init_pos, 0, 2)
    # timeHelper.sleep(2)
    #
    # # land
    cf.land(targetHeight=0.06, duration=3)
