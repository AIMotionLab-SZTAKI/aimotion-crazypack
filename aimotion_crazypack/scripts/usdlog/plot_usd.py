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

def moving_average(a, n=3) :
    ret = np.cumsum(a, dtype=float)
    ret[n:] = ret[n:] - ret[:-n]
    return ret[n - 1:] / n

def filter(a):
    return np.hstack((moving_average(a, 5), 0, 0, 0, 0))

def read_log(filename='/media/crazyfly/UNTITLED/log27'):
    # decode binary log data
    df = cff.decode(filename)
    res = {}
    res["omega_x"] = np.array(df["stateEstimateZ.rateRoll"])
    res["omega_y"] = np.array(df["stateEstimateZ.ratePitch"])
    res["qx"] = np.array(df["stateEstimate.qx"])
    res["qy"] = np.array(df["stateEstimate.qy"])
    res["Mx"] = filter(np.array(df["ctrlFlip.cmd_roll"])/(132/0.0325))
    res["My"] = filter(np.array(df["ctrlFlip.cmd_pitch"])/(132/0.0325))
    res["u0"] = np.array(df["ctrlGeom.u0"])*1000
    res["u1"] = np.array(df["ctrlGeom.u1"])*1000
    res["psi"] = filter(np.array(df["ctrlGeom.psi"]))
    res["ind"] = np.array(df["ctrlGeom.cmd_roll"])
    res["t"] = np.array(df["tick"]) / 1000
    res["x"] = filter(np.array(df["stateEstimate.x"]))
    res["y"] = filter(np.array(df["stateEstimate.y"]))
    res["z"] = filter(np.array(df["stateEstimate.z"]))
    res["xd"] = filter(np.array(df["ctrltarget.x"]))
    res["yd"] = filter(np.array(df["ctrltarget.y"]))
    res["zd"] = filter(np.array(df["ctrltarget.z"]))
    return res


res1 = read_log('log_no_gp')
# res1 = read_log()
res2 = read_log('log_with_gp')
indices1 = [np.abs(grad) > 1e-7 for grad in np.gradient(res1["ind"])]
x = min([indices1.index(i) for i in indices1 if i == True])
indices1[x+250:] = [False for _ in indices1[x+250:]]
indices2 = [np.abs(grad) > 1e-7 for grad in np.gradient(res2["ind"])]
rp_scale = 132000 * 4 / 0.0325

t1 = res1["t"][indices1]
t1 = t1 - t1[0]
t2 = res2["t"][indices2]
t2 = t2 - t2[0]
fig, axs = plt.subplots(5)

axs[0].plot(t1, res1["Mx"][indices1])
axs[1].plot(t1, res1["My"][indices1])
axs[2].plot(t1, res1["u0"][indices1])
axs[3].plot(t1, res1["u1"][indices1])
axs[4].plot(t1[1:], res1["psi"][indices1][1:])
# axs[5].plot(t1, res1["x"][indices1])
# axs[5].plot(t1, res1["y"][indices1])

axs[0].plot(t2, res2["Mx"][indices2])
axs[1].plot(t2, res2["My"][indices2])
axs[2].plot(t2, res2["u0"][indices2])
axs[3].plot(t2, res2["u1"][indices2])
axs[4].plot(t2[1:], res2["psi"][indices2][1:])
# axs[5].plot(t2, res2["x"][indices2])
# axs[5].plot(t2, res2["y"][indices2])

[axs_.set_xlabel("$t$") for axs_ in axs]
axs[0].set_ylabel("$M_x$")
axs[1].set_ylabel("$M_y$")
axs[2].set_ylabel("$\hat\eta_x$")
axs[3].set_ylabel("$\hat\eta_y$")
axs[4].set_ylabel("$\Psi$")

axs[0].legend(('Without GP', 'With GP mean'), loc='upper left')

fig.subplots_adjust(left=0.125,
                    bottom=0.1,
                    right=0.99,
                    top=0.99,
                    hspace=0.8
                    )
[ax.grid(True) for ax in axs]
plt.figure()
plt.plot(res1["x"][indices1], res1["z"][indices1])
plt.plot(res2["x"][indices2], res2["z"][indices2])
plt.show()

# import csv
#
# header1 = (res1.keys())
#
# rows = [v1 for k1,v1 in res1.items()]
#
# with open('csv_log.csv','w') as ofile:
#     wr = csv.writer(ofile, delimiter=',')
#     wr.writerow(header1)
#     wr.writerows(np.array(rows).T)

'''
res = read_log('~/.ros/save/logcf4_gp11.csv')
# res = read_log('~/.ros/logcf4.csv')
indices = [np.abs(grad) > 1e-7 for grad in np.gradient(res["Mx"])]

t = res["t"][indices]
t = t - t[0]
# fig, axs = plt.subplots(3)
axs[0].plot(t, res["Mx"][indices]/rp_scale)
axs[1].plot(t, res["My"][indices]/rp_scale)
axs[2].plot(t, res["u0"][indices])
axs[3].plot(t, res["u1"][indices])
axs[4].plot(t, res["psi"][indices])
axs[0].set_ylabel('$M_x$')
axs[0].legend(('Nominal geom control', 'Geom control + GP mean'))
axs[1].set_ylabel('$M_y$')
axs[2].set_ylabel('$\hat\eta_x$')
axs[3].set_ylabel('$\hat\eta_y$')
axs[4].set_ylim([0, 0.1])
axs[4].set_ylabel('$\Psi$')
'''

