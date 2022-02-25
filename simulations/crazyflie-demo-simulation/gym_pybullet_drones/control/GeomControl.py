import math
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.control.BaseControl import BaseControl
from gym_pybullet_drones.envs.BaseAviary import DroneModel, BaseAviary


class GeomControl(BaseControl):
    """Geometric control class for Crazyflies.

    Based on work conducted at UTIAS' DSL by SiQi Zhou and James Xu.

    """

    ################################################################################

    def __init__(self,
                 env: BaseAviary
                 ):
        """Geometric control initialization.

        Parameters
        ----------
        env : BaseAviary
            The simulation environment to control.

        """
        super().__init__(env=env)
        if self.DRONE_MODEL != DroneModel.CF2X and self.DRONE_MODEL != DroneModel.CF2P:
            print("[ERROR] in DSLPIDControl.__init__(), DSLPIDControl requires DroneModel.CF2X or DroneModel.CF2P")
            exit()
        self.k_R = 8e-2*np.eye(3)  # 1
        self.k_w = 2e-3*np.eye(3)  # 0.04
        self.k_r = np.diag([0.5, 0.5, 1.25])
        self.k_v = np.diag([0.2, 0.2, 0.8])
        self.inertia = np.diag([1.4e-5, 1.4e-5, 2.17e-5])
        self.mass = 0.027
        self.gravity = 9.81
        self.PWM2RPM_SCALE = 0.2685
        self.PWM2RPM_CONST = 4070.3
        self.MIN_PWM = 20000
        self.MAX_PWM = 65535
        if self.DRONE_MODEL == DroneModel.CF2X:
            self.MIXER_MATRIX = np.array([[.5, -.5, -1], [.5, .5, 1], [-.5, .5, -1], [-.5, -.5, 1]])
        elif self.DRONE_MODEL == DroneModel.CF2P:
            self.MIXER_MATRIX = np.array([[0, -1, -1], [+1, 0, 1], [0, 1, -1], [-1, 0, 1]])

        self.l = 0.046/np.sqrt(2)
        self.k = 3.16e-10*(60/2/np.pi)**2
        self.b = 7.94e-12*(60/2/np.pi)**2
        self.INP = np.array([[1, 1, 1, 1], [self.l, self.l, -self.l, -self.l], [-self.l, self.l, self.l, -self.l],
                             [-self.b/self.k, self.b/self.k, -self.b/self.k, self.b/self.k]])
        self.thrust_data = np.loadtxt('../files/thrust.csv', dtype=float, delimiter=',')
        self.pos_data = np.loadtxt('../files/pos.csv', dtype=float, delimiter=',')
        self.reset()


    ################################################################################

    def reset(self):
        """Resets the control classes.

        The previous step's and integral errors for both position and attitude are set to zero.

        """
        super().reset()
        #### Initialized PID control variables #####################
        self.last_pos_e = np.zeros(3)
        self.integral_pos_e = np.zeros(3)
        self.last_rpy_e = np.zeros(3)
        self.integral_rpy_e = np.zeros(3)

    ################################################################################

    def computeControl(self,
                       control_timestep,
                       cur_pos,
                       cur_quat,
                       cur_vel,
                       cur_ang_vel,
                       target_pos,
                       target_rpy=np.zeros(3),
                       target_vel=np.zeros(3),
                       target_ang_vel=np.zeros(3)
                       ):
        """Computes the PID control action (as RPMs) for a single drone.

        This methods sequentially calls `_dslPIDPositionControl()` and `_dslPIDAttitudeControl()`.

        Parameters
        ----------
        control_timestep : float
            The time step at which control is computed.
        cur_pos : ndarray
            (3,1)-shaped array of floats containing the current position.
        cur_quat : ndarray
            (4,1)-shaped array of floats containing the current orientation as a quaternion.
        cur_vel : ndarray
            (3,1)-shaped array of floats containing the current velocity.
        cur_ang_vel : ndarray
            (3,1)-shaped array of floats containing the current angular velocity.
        target_pos : ndarray
            (3,1)-shaped array of floats containing the desired position.
        target_rpy : ndarray, optional
            (3,1)-shaped array of floats containing the desired orientation as roll, pitch, yaw.
        target_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired velocity.
        target_ang_vel : ndarray, optional
            (3,1)-shaped array of floats containing the desired angular velocity.

        Returns
        -------
        ndarray
            (4,1)-shaped array of integers containing the RPMs to apply to each of the 4 motors.
        ndarray
            (3,1)-shaped array of floats containing the current XYZ position error.
        float
            The current yaw error.

        """
        self.control_counter += 1
        ###################
        p_target = np.zeros(3)

        if 0.8/control_timestep > self.control_counter > 0.1/control_timestep:
            q0 = 1.99/(1+np.exp(-20*(control_timestep*(self.control_counter-0.1/control_timestep)-0.45)))-0.995
            dq0 = (39.8 * np.exp(-20 * (control_timestep * (self.control_counter - 0.1/control_timestep) - 0.45))) / (
                        np.exp(-20 * (control_timestep * (self.control_counter - 0.1/control_timestep) - 0.45)) + 1) ** 2
            target_ang_vel[1] = -2*dq0/np.sqrt(1-q0**2)
            q2 = np.sqrt(1-q0**2)
            target_quat = np.array([0, q2, 0, q0])
            target_rotation = (Rotation.from_quat(target_quat)).as_matrix()
            cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)
            rot_e = 1 / 2 / np.sqrt(1 + np.trace(np.dot((target_rotation.transpose()), cur_rotation))) * \
                    self._veemap(np.dot((target_rotation.transpose()), cur_rotation) - np.dot(cur_rotation.transpose(),
                                                                                              target_rotation))
            ang_vel_e = cur_ang_vel - np.dot(np.dot(cur_rotation.transpose(), target_rotation), target_ang_vel)
            target_torques = -self.k_R @ rot_e - self.k_w @ ang_vel_e + \
                             np.cross(cur_ang_vel, np.dot(self.inertia, cur_ang_vel))
            # thrust = 0.22 * np.cos(2 * np.pi * control_timestep * (self.control_counter - 48) / 0.9) + 0.33

            thrust = self.thrust_data[self.control_counter - 49]
            p_target[0] = -self.pos_data[self.control_counter - 49, 0]
            p_target[1] = 0
            p_target[2] = self.pos_data[self.control_counter - 49, 1] + 1
            pos_e = cur_pos - p_target
            vel_e = cur_vel - target_vel
            ddr_d = np.zeros(3)
            temp = -self.k_r @ pos_e - self.k_v @ vel_e + self.mass * self.gravity * np.array(
                [0, 0, 1]) + self.mass * ddr_d
            # thrust = np.dot(temp, np.dot(cur_rotation, np.array([0, 0, 1])))

        ##########################
        else:
            p_target = target_pos
            pos_e = cur_pos - target_pos
            vel_e = cur_vel - target_vel
            ddr_d = np.array([0, 0, 0])
            target_yaw = target_rpy[2]

            temp = -self.k_r @ pos_e - self.k_v @ vel_e + self.mass * self.gravity * np.array(
                [0, 0, 1]) + self.mass * ddr_d
            r3 = temp / np.linalg.norm(temp)
            r2 = np.cross(r3, np.array([np.cos(target_yaw), np.sin(target_yaw), 0])) / \
                 np.linalg.norm(np.cross(r3, np.array([np.cos(target_yaw), np.sin(target_yaw), 0])))
            r1 = np.cross(r2, r3)
            target_rotation = np.array([r1, r2, r3]).transpose()

            cur_rotation = np.array(p.getMatrixFromQuaternion(cur_quat)).reshape(3, 3)

            rot_e = 1 / 2 / np.sqrt(1 + np.trace(np.dot((target_rotation.transpose()), cur_rotation))) * \
                    self._veemap(np.dot((target_rotation.transpose()), cur_rotation) - np.dot(cur_rotation.transpose(),
                                                                                              target_rotation))
            ang_vel_e = cur_ang_vel - np.dot(np.dot(cur_rotation.transpose(), target_rotation), target_ang_vel)
            target_torques = -self.k_R @ rot_e - self.k_w @ ang_vel_e + \
                             np.cross(cur_ang_vel, np.dot(self.inertia, cur_ang_vel))

            error = 0.5*np.trace(np.eye(3) - np.dot((target_rotation.transpose()), cur_rotation))
            # print(error)
            thrust = np.dot(temp, np.dot(cur_rotation, np.array([0, 0, 1])))

        tt = np.zeros(4)
        tt[0] = thrust
        tt[1:4] = target_torques
        f = np.dot(np.linalg.inv(self.INP), tt)
        f = np.clip(f, 0, 0.16)
        rpm = np.sqrt(f/self.k)*30/np.pi
        # rpm = np.clip(rpm, 0, 21666)

        actual_torque = np.dot(self.INP, self.k*(rpm*np.pi/30)**2)[1:4]

        target_euler = np.array([0, 0, 0])  # (Rotation.from_matrix(target_rotation)).as_euler('XYZ', degrees=False)

        cur_rpy = p.getEulerFromQuaternion(cur_quat)

        return rpm, p_target, target_euler[1]

    def _veemap(self, A):
        a = np.zeros(3)
        a[0] = A[2, 1]
        a[1] = A[0, 2]
        a[2] = A[1, 0]
        return a

    def _hatmap(self, a):
        A = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
        return A