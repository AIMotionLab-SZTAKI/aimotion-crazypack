import numpy as np
import matplotlib.pyplot as plt

def polyeval(t, coef):
    return sum([coef_*t**i for i, coef_ in enumerate(coef)])


if __name__ == "__main__":
    data = np.loadtxt("csv/vehicle02.csv", dtype=None, delimiter=',', encoding=None, skiprows=1)
    durations = data[:, 0]
    xcoef = data[:, 1:9]
    ycoef = data[:, 9:17]
    zcoef = data[:, 17:25]
    yawcoef = data[:, 25:33]
    values = []
    dur = np.hstack((0, np.cumsum(durations)))
    dur = np.hstack((0, durations))

    tspan = [np.linspace(dur[i], dur[i+1], int(durations[i]*10)) for i in range(len(dur)-1)]
    ytraj = [polyeval(tspan_, ycoef[0, :]) for tspan_ in tspan[0]]
    # print((xcoef))
    plt.plot(tspan[0], ytraj)
    plt.show()
