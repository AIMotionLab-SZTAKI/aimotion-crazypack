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


res = read_log('~/.ros/save/logcf4_nom11.csv')
indices = [np.abs(grad) > 1e-7 for grad in np.gradient(res["Mx"])]
rp_scale = 7345302.2678

t = res["t"][indices]
t = t - t[0]
fig, axs = plt.subplots(5)
axs[0].plot(t, res["Mx"][indices]/rp_scale)
axs[1].plot(t, res["My"][indices]/rp_scale)
axs[2].plot(t, res["u0"][indices])
axs[3].plot(t, res["u1"][indices])
axs[4].plot(t, res["psi"][indices])

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
[ax.grid(True) for ax in axs]
