import autograd.numpy as np
import math
from autograd import grad
import matplotlib.pyplot as plt
# from numpy import trapz
from math import hypot
from numpy import interp

from casadi import SX, Function, vertcat, cos, sin

class FrenetPath(object):
    def __init__(self, x0 : float, xf : float, N : int):
        # x0, xf, N, = -5, 5, 100
        
        self.x0 = x0
        self.xf = xf
        self.N = N

        self.f_d = grad(self.f)
        self.f_dd = grad(self.f_d)
        self.f_sx_to_x = self.get_f_sx_to_x()
        self.f_c = self.curvature
        # self.f_c_d = grad(self.curvature)

    # The path function, the derivatives and the curvature
    def f(self, x):
        # return 0*x
        # return 10 / (1 + np.exp(-x))
        return np.sin(x/(math.pi/2.0))
    # def derivative(self, x):
    #     return self.f_d(x)
    # def second_derivative(self, x):
    #     return self.f_dd(x)
    def curvature(self, x):
        # https://www.johndcook.com/blog/2018/03/30/curvature-and-automatic-differentiation/
        return abs(self.f_dd(x)) * (1 + self.f_d(x)**2)**-1.5
        # return self.f_dd(x) * (1 + self.f_d(x)**2)**-1.5

    
    def path_length(self, x0 : float = None, xf : float = None, tol : float = 1e-2):
        """Compute the path length of function f(x) for x0 <= x <= xf. Stop
        when two consecutive approximations are closer than the value of tol.
        """
        # https://stackoverflow.com/questions/46098157/how-to-calculate-the-length-of-a-curve-of-a-math-function

        if x0 is None:
            x0 = self.x0
        if xf is None:
            xf = self.xf
            
        n_steps, old_length, length = 1, 1.0e20, 1.0e10

        while abs(old_length - length) >= tol:
            n_steps *= 2
            fx1 = self.f(x0)
            xdel = (xf - x0) / n_steps  # space between x-values
            old_length = length
            length = 0
            for i in range(1, n_steps + 1):
                fx0 = fx1  # previous function value
                fx1 = self.f(x0 + i * (xf - x0) / n_steps)  # new function value
                length += hypot(xdel, fx1 - fx0)  # length of small line segment
        return length
    
    def get_f_sx_to_x(self):
        """
        Its output is a polinom, which maps sx to x: x = f(sx)
        The results are only valid between x0 and xf, the interval on which
        the path is defined.
        We will call this polinome in the function sx_to_x, when transforming
        from sx to x.
        """
        x = np.linspace(self.x0, self.xf, self.N)
        sx = [self.path_length(x0 = self.x0, xf = x_) for x_ in x]
        self.sx = sx
        z = np.polyfit(x = sx, y = x, deg = self.N + 1)
        return np.poly1d(z)
    def sx_to_x(self, sx):
        """     
        Transforming sx to x.
        """
        return self.f_sx_to_x(sx)
    
    def t_to_sx(self, t):
        """ remap function, so that we can input t = [0, 1] instead
        of a value between 0 and sx_max
        """
        return interp(t,[0,1],[self.sx[0],self.sx[-1]])
        
    def t_to_xy(self, t):
        """
        Transforming t to x and y coodinates.
        """
        sx = self.t_to_sx(t)
        x = self.f_sx_to_x(sx)
        y = self.f(x)
        return x, y
    
        
    
    # Simulation related stuff
    def simulation_step(self):
        """
        Performs a single step.
        """
        # Moove x0
        t = 0.1
        x, y = self.t_to_xy(t)
        self.x0 = x
        # Decrease N
        self.N = self.N - 1
        # Recalculate everything
        
        self.f_d = grad(self.f)
        self.f_dd = grad(self.f_d)
        self.f_sx_to_x = self.get_f_sx_to_x()
        self.f_c = self.curvature
        
        return self
        
    def s_dot(self, T, t):
        """
        The frenet coodinate system mooves along the path at constant speed.
        """
        if t < 1 and t >= 0:
            return self.path_length() / T
        else:
            # At the end the frenet frame stops mooving.
            return 0
        
    # def omega_c(self, x, T, t):
    #     return self.f_c(x) * self.s_dot(T, t)

    # Plot
    def plot_rotation(self, x, y, theta):
        x_new = x * cos(theta) - y * sin(theta)
        y_new = x * sin(theta) + y * cos(theta)
        return x_new, y_new
    def plot_path(self, ax, t):
        import numpy as np
        # Plottin entire path
        x = np.linspace(self.x0, self.xf, self.N)
        y = [self.f(x_) for x_ in x]
        ax.plot(x, y, label='path', linestyle = ':', color = 'gray', zorder = 2)
        
        # Plotting current position of the frenet coordinate system
        x_current, y_current = self.t_to_xy(t)
        x_, _ = self.t_to_xy(t)
        theta_c = math.atan2(self.f_d(float(x_)), 1)
        
        rot_x, rot_y = self.plot_rotation(0.1, 0, theta_c)
        ax.arrow(x_current, y_current, rot_x, rot_y, head_width=0.01, head_length=0.02, fc='r', ec='r', zorder = 3)
        rot_x, rot_y = self.plot_rotation(0.1, 0, theta_c + np.pi/2)
        ax.arrow(x_current, y_current, rot_x, rot_y, head_width=0.01, head_length=0.02, fc='r', ec='r', zorder = 3)
        
        
        # ax.plot(x_current, y_current, 'go', label='curr. pos. of frame')
        # ax.annotate('double-headed arrow', xy=(0.45,0.5), xytext=(0.01,0.5),
        #     arrowprops={'arrowstyle': '<->'}, va='center')
        # ax.plot(arrow_points[:,0], arrow_points[:,1], alpha = 0.5, linewidth = 1, color = color)
        # arrow(0,0,val*vec[ix],val*vec[iy],head_width=0.05*(dx*dy/4)**0.5,fc=c,ec=c)
        
        return ax, x_current, y_current
        
        
    def plot_curvature(self):
        x = np.linspace(self.x0, self.xf, self.N)
        y = [self.curvature(x_) for x_ in x]
        plt.plot(x, y)
        
    def frenet_to_inertial(self, p, q, t):
        """
        Takes a position in the frenet frame and transforms it to
        xy coordinates, given the frenet frame is at a position, defined by t.
        """
        x, y = self.t_to_xy(t)
        theta_c = math.atan2(self.f_d(x), 1)
        x_ = p * cos(theta_c) - q * sin(theta_c)
        y_ = p * sin(theta_c) + q * cos(theta_c)
        
        return x + x_, y + y_
    
    
    def inertial_to_frenet(self, x, y, t):
        """
        Takes a position in the inertial frame and transforms it to
        pq coordinates in the frenet frame, given the frenet frame is at a 
        position, defined by t.
        """
        x_frenet, y_frenet = self.t_to_xy(t)
        theta_c = math.atan2(self.f_d(x_frenet), 1)
        x_rel = x - x_frenet
        y_rel = y - y_frenet
        
        p = x_rel * cos(theta_c) + y_rel * sin(theta_c)
        q =-x_rel * sin(theta_c) + y_rel * cos(theta_c)
        
        return p, q

"""
Kinematics & Integrator
"""      

def discrete_kinematics():
    """
    Defines the kinematics of the system
    """
    
    # Vehicle
    # system state
    p = SX.sym('p')
    q = SX.sym('q')
    v = SX.sym('v')
    theta_m = SX.sym('theta_m')
    omega_m = SX.sym('omega_m')
    # system input
    a = SX.sym('a')
    beta_m = SX.sym('beta_m')
    
    # Frenet frame
    # frenet state
    s = SX.sym('s')
    v_s = SX.sym('v_s')
    theta_c = SX.sym('theta_c')
    # frenet input
    a_s = SX.sym('a_s')
    curvature = SX.sym('curvature')
    
    """
    Additional equations for simplifications
    """
    theta = theta_m - theta_c    
    
    """
    State equations
    """
    p_dot = -v_s * (1 - curvature * q) + v * cos(theta) # curvature is an input
    q_dot = -curvature * v_s * p + v * sin(theta) # curvature is an input
    v_dot = a # a is an input
    theta_m_dot = omega_m
    omega_m_dot = beta_m # beta_m is an input
    s_dot = v_s
    v_s_dot = a_s # a_s is an input
    theta_c_dot = curvature * v_s # curvature is an input
    
    
    """
    Function inputs and outputs
    """
    # Inputs
    state_m = [p, q, v, theta_m, omega_m]
    state_m_input = [a, beta_m]
    state_c = [s, v_s, theta_c]
    state_c_input = [a_s, curvature]
    # Outputs
    state_m_dot = [p_dot, q_dot, v_dot, theta_m_dot, omega_m_dot]
    state_c_dot = [s_dot, v_s_dot, theta_c_dot]
    
    
    """
    Defining the function
    """
    f = Function('f', [vertcat(*state_m), vertcat(*state_c), vertcat(*state_m_input), vertcat(*state_c_input)], \
                 [vertcat(*state_m_dot), vertcat(*state_c_dot)], \
                 ['state_m', 'state_c', 'state_m_input', 'state_c_input'],
                 ['state_m_dot', 'state_c_dot'] \
                )
    
    
    state_m = SX.sym('state_m', 5)
    state_c = SX.sym('state_c', 3)
    state_m_input = SX.sym('state_m_input', 2)
    state_c_input = SX.sym('state_c_input', 2)
    
    
    [state_m_dot, state_c_dot] = f(state_m,
                                   state_c,
                                   state_m_input,
                                   state_c_input)
    
    dt = SX.sym('dt', 1)
    state_m_next = state_m + dt * state_m_dot
    F = Function('F',
                 [state_m, state_c, state_m_input, state_c_input, dt],
                 [state_m_next],
                 ['state_m', 'state_c', 'state_m_input', 'state_c_input', 'dt'],
                 ['state_m_next'])
        
    return F



def kinematics():
    """
    Defines the kinematics of the system
    """
    
    # Vehicle
    # system state
    p = SX.sym('p')
    q = SX.sym('q')
    v = SX.sym('v')
    theta_m = SX.sym('theta_m')
    omega_m = SX.sym('omega_m')
    # system input
    a = SX.sym('a')
    beta_m = SX.sym('beta_m')
    
    # Frenet frame
    # frenet state
    s = SX.sym('s')
    v_s = SX.sym('v_s')
    theta_c = SX.sym('theta_c')
    # frenet input
    a_s = SX.sym('a_s')
    curvature = SX.sym('curvature')
    
    """
    Additional equations for simplifications
    """
    theta = theta_m - theta_c    
    
    """
    State equations
    """
    p_dot = -v_s * (1 - curvature * q) + v * cos(theta) # curvature is an input
    q_dot = -curvature * v_s * p + v * sin(theta) # curvature is an input
    v_dot = a # a is an input
    theta_m_dot = omega_m
    omega_m_dot = beta_m # beta_m is an input
    s_dot = v_s
    v_s_dot = a_s # a_s is an input
    theta_c_dot = curvature * v_s # curvature is an input
    
    
    """
    Function inputs and outputs
    """
    # Inputs
    state_m = [p, q, v, theta_m, omega_m]
    state_m_input = [a, beta_m]
    state_c = [s, v_s, theta_c]
    state_c_input = [a_s, curvature]
    # Outputs
    state_m_dot = [p_dot, q_dot, v_dot, theta_m_dot, omega_m_dot]
    state_c_dot = [s_dot, v_s_dot, theta_c_dot]
    
    
    """
    Defining the function
    """
    f = Function('f', [vertcat(*state_m), vertcat(*state_c), vertcat(*state_m_input), vertcat(*state_c_input)], \
                 [vertcat(*state_m_dot), vertcat(*state_c_dot)], \
                 ['state_m', 'state_c', 'state_m_input', 'state_c_input'],
                 ['state_m_dot', 'state_c_dot'] \
                )
    
    return f
    

    

def integrator():
    """
    Integrates the system state using a simple forward Euler integration process.
    """
    dt = SX.sym('dt')
    x0 = SX.sym('x0', 5)
    x_dot = SX.sym('x_dot', 5)
    xf = SX.sym('xf', 5)
    # Forward Euler integration
    xf = x0 + dt * x_dot
    
    # Defining the function
    f = Function('state_integrator', [x0, x_dot, dt], [xf], ['x0', 'x_dot', 'dt'], ['xf'] )
    
    return f




def fold(vector : list, n_vehicles : int, n_states : int):
    """A helper function, used in other functions, where the format of the
    data has to be changed. Its "inverse" function is unfold().
    fold() takes a vector of shape(1, n_vehicles * 4 * N) and folds it into
    a matrix of shape(n_vehicles, 4 * N).

    The resulting rows of the matrix contain data associated with the given
    neighbour agent. This is the format in which data is stored in the messages or
    this format is used when some computation is taking place, like updating
    the lambda values.
    """

    # Takes a vector
    # Unfolds into np.array(n_vehicles, N * 4)
    matrix = np.array(vector).reshape(n_vehicles, -1) * 0
    tmp_matrix = np.array(vector).reshape(-1, n_states)

    for i in range(n_vehicles):
        matrix[i, :] = tmp_matrix[i::n_vehicles, :].reshape(1, -1)

    return matrix

# state_solution_matrix = fold(state_solution, n_vehicles, n_states).tolist()
# input_solution_matrix = fold(input_solution, n_vehicles, n_inputs).tolist()