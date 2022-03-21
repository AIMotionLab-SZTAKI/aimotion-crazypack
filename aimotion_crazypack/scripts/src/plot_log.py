import numpy as np
from matplotlib import pyplot as plt
import pandas


def read_log(filename='~/.ros/save/logcf4_gp.csv'):
    df = pandas.read_csv(filename)

    res = {}
    # res["omega_x"] = np.array(df["stateEstimateZ.rateRoll"])
    # res["omega_y"] = np.array(df["stateEstimateZ.ratePitch"])
    # res["qx"] = np.array(df["stateEstimate.qx"])
    # res["qy"] = np.array(df["stateEstimate.qy"])
    res["Mx"] = np.array(df["ctrlGeom.cmd_roll"])
    res["My"] = np.array(df["ctrlGeom.cmd_pitch"])
    res["u0"] = np.array(df["ctrlGeom.u0"])
    res["u1"] = np.array(df["ctrlGeom.u1"])
    res["psi"] = np.array(df["ctrlGeom.psi"])
    res["t"] = np.array(df["time"])
    return res


res = read_log('~/.ros/save/logcf4_gp3.csv')
indices = [np.abs(grad) > 1e-7 for grad in np.gradient(res["Mx"])]
rp_scale = 7345302.2678

t = res["t"][indices]
t = t - t[0]
fig, axs = plt.subplots(3)
axs[0].plot(t, res["Mx"][indices]/rp_scale, 'b-')
axs[0].plot(t, res["My"][indices]/rp_scale, 'r-')
axs[1].plot(t, res["u0"][indices], 'b-')
axs[1].plot(t, res["u1"][indices], 'r-')
axs[2].plot(t, res["psi"][indices], 'b-')

res = read_log('~/.ros/save/logcf4_nom2.csv')
indices = [np.abs(grad) > 1e-7 for grad in np.gradient(res["Mx"])]

t = res["t"][indices]
t = t - t[0]
# fig, axs = plt.subplots(3)
axs[0].plot(t, res["Mx"][indices]/rp_scale, 'b--')
axs[0].plot(t, res["My"][indices]/rp_scale, 'r--')
axs[1].plot(t, res["u0"][indices], 'b--')
# axs[1].plot(t, res["u1"][indices], 'r.')
axs[2].plot(t, res["psi"][indices], 'b--')
