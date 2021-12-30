import numpy as np
import matplotlib.pyplot as plt
# from src.spline import BSpline, BSplineBasis
import scipy.interpolate as interp

def polyeval(t, coef):
    return sum([coef_*t**i for i, coef_ in enumerate(coef)])


if __name__ == "__main__":
    knot_interval = 0.2
    knots = np.tile(
        np.hstack((np.zeros(3), np.linspace(0, 1, 9), np.ones(3))),
        (4, 1))
    coeffs = np.zeros((4, 11))
    coeffs[0, :] = np.hstack((0, np.linspace(0, 0.4, 5),
                                   0.4 * np.ones(5)))
    degree = [3, 3, 3, 3]
    spl = [interp.BSpline(knots[i, :], coeffs[i, :], degree[i]) for i in range(4)]
    timespan = np.linspace(0, 1, 100)
    pp = [interp.PPoly.from_spline(spl_) for spl_ in spl]

    header = ["duration","x^0","x^1","x^2","x^3","x^4","x^5","x^6","x^7","y^0","y^1","y^2","y^3","y^4","y^5","y^6",
              "y^7","z^0","z^1","z^2","z^3","z^4","z^5","z^6","z^7","yaw^0","yaw^1","yaw^2","yaw^3","yaw^4","yaw^5",
              "yaw^6","yaw^7"]
    import csv
    # open the file in the write mode
    with open('csvname.csv', 'w', newline='') as f:
        # create the csv writer
        writer = csv.writer(f)
        # write a row to the csv file
        writer.writerow(header)
        last_bp = pp[0].x[0]
        zeros = [0, 0, 0, 0]
        for i, bp in enumerate(pp[0].x):
            duration = bp - last_bp
            if duration > 0 and i > 0:
                ppcoef = []
                [ppcoef.extend(pp[k].c[:, i - 1].tolist()[::-1] + zeros) for k in range(4)]
                row = [duration] + ppcoef
                writer.writerow(row)
            last_bp = bp

    # plt.plot(timespan, pp(timespan))
    # plt.show()

    # data = np.loadtxt("csv/vehicle02.csv", dtype=None, delimiter=',', encoding=None, skiprows=1)
    # durations = data[:, 0]
    # xcoef = data[:, 1:9]
    # ycoef = data[:, 9:17]
    # zcoef = data[:, 17:25]
    # yawcoef = data[:, 25:33]
    # values = []
    # dur = np.hstack((0, np.cumsum(durations)))
    # dur = np.hstack((0, durations))
    #
    # tspan = [np.linspace(dur[i], dur[i+1], int(durations[i]*10)) for i in range(len(dur)-1)]
    # ytraj = [polyeval(tspan_, ycoef[0, :]) for tspan_ in tspan[0]]
    # # print((xcoef))
    # plt.plot(tspan[0], ytraj)
    # plt.show()
