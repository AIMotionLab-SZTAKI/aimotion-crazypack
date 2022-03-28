############ Steps #######
# 1. Read log csv
# 2. Get flip start and end
# 3. Prepare training inputs and outputs
# 4. Fit GP
# 5. Create lookup table
# 6. Send lookup table to Crazyflie

import numpy as np
import pandas
import matplotlib.pyplot as plt
import torch
import gpytorch
import sys
sys.path.append('../../../aimotion-crazypack-firmware/tools/usdlog')
import CF_functions as cff

def RBFkernel(x1, x2, lam, sf2):
    L = np.linalg.inv(np.diag(lam)**2)
    K = sf2*np.exp(-0.5*(x1-x2).T @ L @ (x1-x2))
    return K

def read_log_from_tau(filename="/home/crazyfly/.ros/logcf4.csv"):
    df = pandas.read_csv(filename)
    omega_x = np.array(df["stateEstimateZ.rateRoll"])
    omega_y = np.array(df["stateEstimateZ.ratePitch"])
    qx = np.array(df["stateEstimate.qx"])
    qy = np.array(df["stateEstimate.qy"])
    Mx = np.array(df["ctrlGeom.cmd_roll"])
    My = np.array(df["ctrlGeom.cmd_pitch"])

    res = {}
    res["omega_x"] = omega_x
    res["omega_y"] = omega_y
    res["qx"] = qx
    res["qy"] = qy
    res["Mx"] = Mx
    res["My"] = My
    res["t"] = np.array(df["time"])
    return res

def read_log_from_tau_usd(filename="../usdlog/log16.csv"):
    df = cff.decode(filename)
    omega_x = np.array(df["stateEstimateZ.rateRoll"])
    omega_y = np.array(df["stateEstimateZ.ratePitch"])
    qx = np.array(df["stateEstimate.qx"])
    qy = np.array(df["stateEstimate.qy"])
    Mx = np.array(df["ctrlFlip.cmd_roll"])
    My = np.array(df["ctrlFlip.cmd_pitch"])

    res = {}
    res["omega_x"] = omega_x
    res["omega_y"] = omega_y
    res["qx"] = qx
    res["qy"] = qy
    res["Mx"] = Mx
    res["My"] = My
    res["t"] = np.array(df["tick"]) / 1000
    res["ind"] = np.array(df["ctrlGeom.cmd_roll"])

    return res

def printC(mat):   ######## Print numpy array in C array format ############
    if mat.ndim == 1:
        code='{'
        for j in range(mat.shape[0]):
            code = code + "{:.3f}".format(mat[j]) + ','
        code = code[0:-1] + '}'
    else:
        code = '{{'
        for i in range(mat.shape[0]):
            for j in range(mat.shape[1]):
                code = code + "{:.3f}".format(mat[i][j]) + ','
            code = code[0:-1] + '},\n{'
        code = code[0:-3] + '}'
    return code

class ExactGPModel(gpytorch.models.ExactGP):
    def __init__(self, train_x, train_y, likelihood):
        super(ExactGPModel, self).__init__(train_x, train_y, likelihood)
        self.mean_module = gpytorch.means.ZeroMean()
        self.covar_module = gpytorch.kernels.ScaleKernel(gpytorch.kernels.RBFKernel(ard_num_dims=train_x.size(1)))

    def forward(self, x):
        mean_x = self.mean_module(x)
        covar_x = self.covar_module(x)
        return gpytorch.distributions.MultivariateNormal(mean_x, covar_x)

rp_scale = 132000*4/0.0325
yaw_scale = 132000*4/0.025
# res1 = read_log_from_tau(filename="/home/crazyfly/.ros/save/logcf4_train.csv")
# res = read_log_from_tau(filename="/home/crazyfly/.ros/logcf4.csv")
res = read_log_from_tau_usd(filename="/media/crazyfly/UNTITLED/log05")
# res2 = read_log_from_tau(filename="/home/crazyfly/.ros/logcf4.csv")
# res2 = read_log_from_tau(filename="/home/crazyfly/.ros/save/logcf4_train2.csv")
# res = {}
# for elem in res1:
#     res[elem] = np.hstack((res1[elem], res2[elem]))
indices = [np.abs(grad) > 1e-7 for grad in np.gradient(res["ind"])]
Mx = res["Mx"][indices]/rp_scale
My = res["My"][indices]/rp_scale
inertia = np.diag([1.4e-5, 1.4e-5, 2.17e-5])
ang_vel = np.vstack((res["omega_x"][indices], res["omega_y"][indices], np.zeros_like(res["omega_x"][indices])))/1000
ang_accel = np.zeros_like(ang_vel)
for i in range(3):
    ang_accel[i, :] = np.gradient(ang_vel[i, :], 0.004)
M2 = np.zeros_like(ang_accel)
torque = np.vstack((Mx, My, np.zeros_like(Mx)))
for i in range(ang_accel.shape[1]):
    M2[:, i] = inertia @ ang_accel[:, i] + np.cross(ang_vel[:, i], inertia @ ang_vel[:, i])

# plt.figure()
# plt.plot(res["omega_y"][indices], torque.T - M2.T)
plt.figure()
plt.plot(res["t"][indices], M2.T[:, 1])
plt.plot(res["t"][indices], torque.T[:, 1])
plt.show()

# train_x = np.vstack((res["qx"][indices], res["qy"][indices], res["omega_x"][indices]/10000, res["omega_y"][indices]/10000)).T
# train_y = [(torque - M2)[0, :].T, (torque - M2)[1, :].T]
# train_y = [train_yi * 200 + 0.01 * np.random.randn(train_yi.shape[0]) for train_yi in train_y]
#
# train_x = torch.from_numpy(train_x).float()
# train_y = [torch.from_numpy(train_yi).float() for train_yi in train_y]
#
# likelihood = [gpytorch.likelihoods.GaussianLikelihood() for _ in range(2)]
# model = [ExactGPModel(train_x, train_y[i], likelihood[i]) for i in range(2)]
#
# training_iter = 200
# # Find optimal model hyperparameters
# for i in range(2):
#     model[i].train()
#     likelihood[i].train()
#
#     # Use the adam optimizer
#     optimizer = torch.optim.Adam(model[i].parameters(), lr=0.1)  # Includes GaussianLikelihood parameters
#
#     # "Loss" for GPs - the marginal log likelihood
#     mll = gpytorch.mlls.ExactMarginalLogLikelihood(likelihood[i], model[i])
#
#     for iter in range(training_iter):
#         # Zero gradients from previous iteration
#         optimizer.zero_grad()
#         # Output from model
#         output = model[i](train_x)
#         # Calc loss and backprop gradients
#         loss = -mll(output, train_y[i])
#         loss.backward()
#         # print('Iter %d/%d - Loss: %.3f   lengthscale: %.3f   noise: %.3f' % (
#         #     iter + 1, training_iter, loss.item(),
#         #     model[i].covar_module.base_kernel.lengthscale.item(),
#         #     model[i].likelihood.noise.item()
#         # ))
#         # print('Iter %d/%d - Loss: %.3f   noise: %.3f' % (
#         #     iter + 1, training_iter, loss.item(),
#         #     model[i].likelihood.noise.item()
#         # ))
#         optimizer.step()
#     print(loss.item())
#
# # sf2 = [m.covar_module.outputscale.detach().numpy() for m in model]
# # lscale = [m.covar_module.base_kernel.lengthscale.detach().numpy() for m in model]
# # se2 = [m.likelihood.noise.detach().numpy()**2 for m in model]
# # X = train_x.numpy()[::3]
# # y = [train_yi.numpy()[::3] for train_yi in train_y]
# # K = [np.zeros((X.shape[0], X.shape[0])) for _ in range(2)]
# # for elem, K1 in enumerate(K):
# #     for i in range(K1.shape[0]):
# #         for j in range(K1.shape[0]):
# #             K1[i, j] = RBFkernel(X[i, :], X[j, :], lscale[elem][0], sf2[elem])
# # alpha = [np.linalg.inv(K1+se2[elem]*np.eye(K1.shape[0])) @ y[elem] for elem, K1 in enumerate(K)]
#
# # print("float X[48][4] = " + printC(X))
# # print("float alpha1[48] = " + printC(alpha[0]))
# # print("float alpha2[48] = " + printC(alpha[1]))
# # print("float lam1[4] = " + printC(1/lscale[0]**2))
# # print("float lam2[4] = " + printC(1/lscale[1]**2))
#
# [m.eval() for m in model]
# [l.eval() for l in likelihood]
# num_x = 5
# num_y = 9
# sum_num = num_x**2 * num_y**2
# qxs = torch.linspace(-0.05, 0.05, num_x)
# qys = torch.linspace(-1, 0.2, num_y)
# wxs = torch.linspace(-0.2, 0.2, num_x)
# wys = torch.linspace(-0.2, 2.3, num_y)
# qxs, qys, wxs, wys = torch.meshgrid(qxs, qys, wxs, wys)
# qxs, qys, wxs, wys = torch.reshape(qxs, (sum_num, 1)), torch.reshape(qys, (sum_num, 1)), \
#                      torch.reshape(wxs, (sum_num, 1)), torch.reshape(wys, (sum_num, 1)),
#
# test_x = torch.cat((qxs, qys, wxs, wys), 1)
# observed_pred = [likelihood[i](model[i](test_x)) for i in range(2)]
# test_y = [torch.reshape(pred.mean, (num_x, num_y, num_x, num_y)) for pred in observed_pred]
# csv_arr = [torch.reshape(test_yi, (sum_num, 1)) for test_yi in test_y]
# # csv_write = 10000*csv_arr[0].detach().numpy().T
# np.savetxt('ltb.csv', 10000*csv_arr[0].detach().numpy().T, "%i", delimiter=',')
# #
# # ltb = np.loadtxt('ltb.csv', delimiter=',')
