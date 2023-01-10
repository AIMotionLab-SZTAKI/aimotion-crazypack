# -*- coding: utf-8 -*-
"""
example on how to plot decoded sensor data from crazyflie
@author: jsschell
"""
import sys

sys.path.append('../../../aimotion-crazypack-firmware/tools/usdlog')
import CF_functions as cff
import matplotlib.pyplot as plt
import re
import argparse
import numpy as np
import pickle


def moving_average(a, n=3):
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n


def filter(a):
    return np.hstack((moving_average(a, 5), 0, 0, 0, 0))


def plot_measurements(measurements, block=True):
    t = (measurements['tick'][:90] - measurements['tick'][0]) / 1000
    # x_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrltarget.x'][:225])
    # z_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrltarget.z'][:225])
    # p_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrlGeom.p_des'][:225])
    # w_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrlGeom.wdy'][:225])
    # pos_ref_meas = np.vstack((x_ref, z_ref, p_ref, w_ref))
    # with open('../pickle/pos_ref_meas.pickle', 'wb') as file:
    #     pickle.dump(pos_ref_meas, file)

    for key in measurements:
        # plt.figure()
        # plt.plot(t, measurements[key][:90])
        # plt.ylabel(key)
        # plt.show(block=False)
        measurements[key] = measurements[key][:90]
    fig, axs = plt.subplots(4, 2)
    axs[0, 0].plot(measurements['stateEstimate.x'], measurements['stateEstimate.z'])
    axs[0, 0].plot(x_ref, z_ref)
    axs[0, 0].set_ylabel('Position tracking')

    axs[1, 0].plot(t, measurements['stateEstimate.x'])
    axs[1, 0].plot(t, measurements['stateEstimate.z'])
    axs[1, 0].plot(t, x_ref)
    axs[1, 0].plot(t, z_ref)
    axs[1, 0].set_ylabel('Position tracking')

    axs[2, 0].plot(t, p_ref)
    axs[2, 0].plot(t, -measurements['stateEstimate.pitch'] * np.pi / 180)
    axs[2, 0].set_ylabel('Pitch tracking')
    # plt.figure()
    # plt.plot(t, measurements['ctrlGeom.wy'])
    # plt.plot(t, w_ref)

    axs[0, 1].plot(t, measurements['ctrlGeom.Mx'])
    axs[0, 1].plot(t, measurements['ctrlGeom.My'])
    axs[0, 1].plot(t, measurements['ctrlGeom.Mz'])
    axs[0, 1].plot(t, measurements['ctrlGeom.thrust'] / 100)
    axs[0, 1].set_ylabel('Control inputs')

    axs[1, 1].plot(t, measurements['ctrlGeom.psi'])
    axs[1, 1].set_ylabel('Attitude error')

    axs[2, 1].plot(t, measurements['ctrlGeom.eRx'])
    axs[2, 1].plot(t, measurements['ctrlGeom.eRy'])
    axs[2, 1].plot(t, measurements['ctrlGeom.eRz'])
    axs[2, 1].set_ylabel('Attitude error')

    if 'ctrlGeom.eta0' in measurements.keys():
        axs[3, 0].plot(t, measurements['ctrlGeom.eta0'])
        axs[3, 0].plot(t, measurements['ctrlGeom.eta1'])

    if 'ctrlGeom.mu0' in measurements.keys():
        axs[3, 1].plot(t, measurements['ctrlGeom.mu0'])
        axs[3, 1].plot(t, measurements['ctrlGeom.mu1'])

    fig.subplots_adjust(left=0.15, bottom=0.05, right=0.99, top=0.98, wspace=0.4, hspace=0.3)
    plt.show(block=block)


if __name__ == '__main__':
    with open('../pickle/pos_ref_meas.pickle', 'rb') as file:
        ref = pickle.load(file)
    x_ref = ref[0, :]
    z_ref = ref[1, :]
    p_ref = ref[2, :]
    w_ref = ref[3, :]
    measurements = cff.decode('/media/crazyfly/UNTITLED/log22')
    # t = (measurements['tick'][:90] - measurements['tick'][0]) / 1000
    # x_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrltarget.x'][:225])
    # z_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrltarget.z'][:225])
    # p_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrlGeom.p_des'][:225])
    # w_ref = np.interp(np.linspace(0, 0.9, 90), t, measurements['ctrlGeom.wdy'][:225])
    # pos_ref_meas = np.vstack((x_ref, z_ref, p_ref, w_ref))
    # with open('../pickle/pos_ref_meas.pickle', 'wb') as file:
    #     pickle.dump(pos_ref_meas, file)

    plot_measurements(measurements, block=True)
    


