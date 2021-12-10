from .spline import BSpline, BSplineBasis
from .frenet_path import FrenetPath

from casadi import MX, SX, Function, vertcat, dot, nlpsol, cos, sin, norm_2

import random
import numpy as np
import matplotlib.pyplot as plt
from .spline_extra import definite_integral
import math


class SplineFitter():
    def __init__(self, knot_intervals : int = 10):
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.w_list, self.g_list, self.P_list = [], [], []

        self.n_dimensions = 2
        self.knot_intervals = knot_intervals
        self.y_min = [-20, -20]
        self.y_max = [20, 20]

    def define_knots(self,degree=3, **kwargs):
        """This function defines the knots and creates the
        B-spline basis function with the prescribed degree.
        Input:
            degree: degree of the B-spline basis functions
            knot_intervals: number of knot intervals
            knots (optional): knot vector. If not given, calculated using the
            number of knot_intervals
        Returns:
            basis: array of B-spline basis functions
            knots: the knot vector
            knot_intervals
        """

        if 'knot_intervals' in kwargs:
            knot_intervals = kwargs['knot_intervals']
            knots = np.r_[np.zeros(degree),
                          np.linspace(0, 1, knot_intervals+1),
                          np.ones(degree)]
        if 'knots' in kwargs:
            knot_intervals = len(knots) - 2*degree - 1
            knots = kwargs['knots']

        basis = BSplineBasis(knots, degree)

        return basis

    def define_splines(self, degree, knot_intervals, n_spl, lower_bound, upper_bound, name = ''):
        """This function defines a set of splines.
        Input:
            basis: the basis function to define the spline with. If not provided,
            self.basis will be used.
            n_spl: number of splines to define
        Returns:
            a B-spline class
        """
        basis = self.define_knots(degree = degree, knot_intervals = knot_intervals)

        splines = []
        for k in range(n_spl):
            coeffs = MX.sym(name[k], len(basis))
            self.w += [coeffs]
            self.w_list += [name[k] for i in range(len(basis))]
            splines += [BSpline(basis, coeffs)]


        for i in range(len(splines)):
            for j in range(splines[i].coeffs.shape[0]):
                self.lbw += [lower_bound[i]]
                self.ubw += [upper_bound[i]]

        return splines

    def define_constraint(self, constraint, lower_bound, upper_bound, constraint_type = 'overall', name = ''):
        """This function defines constraint on the b_spline coefficients
        Input:
            constraint (list): a spline constraint. We will set upper and lower bounds on its coefficients
            lower_bound (list): the lower bound of the coefficients
            upper_bound (list): the upper bound of the coefficients
            constraint_type: 'overall', 'initial', 'final'
        Returns:

        """
        if constraint_type == 'overall':
            for i in range(len(constraint)):
                for j in range(constraint[i].coeffs.shape[0]):
                    self.g += [constraint[i].coeffs[j]]
                    self.g_list += [name]
                    self.lbg += [lower_bound[i]]
                    self.ubg += [upper_bound[i]]
            return self
            
        elif constraint_type == 'time':
            for i in range(len(constraint)):
                # for j in range(constraint[i].coeffs.shape[0]):
                self.g += [constraint[i]]
                for j in range(constraint[i].shape[0]):
                    self.g_list += [name]
                    self.lbg += [lower_bound[i]]
                    self.ubg += [upper_bound[i]]
            return self

        elif constraint_type == 'initial':
            for i in range(len(constraint)):
                self.g += [constraint[i].coeffs[0]] # we restrict the first coefficient
                self.g_list += [name[i] + "_0"]
                self.lbg += [lower_bound[i]]
                self.ubg += [upper_bound[i]]
            return self
        elif constraint_type == 'final':
            for i in range(len(constraint)):
                self.g += [constraint[i].coeffs[-1]] # we restrict the last coefficient
                self.g_list += [name[i] + "_f"]
                self.lbg += [lower_bound[i]]
                self.ubg += [upper_bound[i]]
            return self

        else:
            raise NotImplementedError()

    
    def fitting(self, fx, fy, degree = 3, lbw = [], ubw = []):
        """ This function fits a 3-degree spline on fx and a spline on fy.
        If lbw and ubw are provided, then 'overall' constraints are enforced on
        the spline values up to a degree, specified by the length of the lbw and ubw arrays.
        """
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.w_list, self.g_list, self.P_list = [], [], []


        y = self.define_splines(degree=degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                name=["y"] * self.n_dimensions)

        # Initial position constraint on y
        self.define_constraint(y,
                               [fx[0], fy[0]],
                               [fx[0], fy[0]],
                               constraint_type='initial',
                               name=["y0"] * self.n_dimensions)
        # Final position constraint on y
        self.define_constraint(y,
                               [fx[-1], fy[-1]],
                               [fx[-1], fy[-1]],
                               constraint_type='final',
                               name=["yf"] * self.n_dimensions)
        
        # if len(lbw) > 0 and len(ubw) > 0:
        if lbw != [] and ubw != []:
            # Example: overall constraints on position, velocity and acceleration
            for i in range(len(lbw[0])):
                y_der = [y_.derivative(i) for y_ in y]
                self.define_constraint(y_der,
                                       [lbw[0][i], lbw[0][i]],
                                       [ubw[0][i], ubw[0][i]],
                                       constraint_type='overall',
                                       name=["y_" + "d" * i + "_overall"] * self.n_dimensions)
                
                                        # [lbw[0][i] * 10, lbw[0][i] * 10],
                                        # [ubw[0][i] * 10, ubw[0][i] * 10],
                                       # [lbw[0][i], lbw[0][i]],
                                       # [ubw[0][i], ubw[0][i]],
                                       
            # Also, we assume, that the first derivative's initial & final value is zero
            # meaning: velocity at the start & at the beginning is zero.
            # But! this might not be what we want, so be carefull!
            y_der = [y_.derivative() for y_ in y]
            self.define_constraint(y_der,
                                   [0, 0],
                                   [0, 0],
                                   constraint_type='initial',
                                   name=["y_" + "d" + "_initial"] * self.n_dimensions)
            self.define_constraint(y_der,
                                   [0, 0],
                                   [0, 0],
                                   constraint_type='final',
                                   name=["y_" + "d" + "_final"] * self.n_dimensions)
            
            # Nope! We will add it to the 2nd derivative, aka to the acceleration as well ;)
            y_der = [y_.derivative(2) for y_ in y]
            self.define_constraint(y_der,
                                    [0, 0],
                                    [0, 0],
                                    constraint_type='initial',
                                    name=["y_" + "dd" + "_initial"] * self.n_dimensions)
            self.define_constraint(y_der,
                                    [0, 0],
                                    [0, 0],
                                    constraint_type='final',
                                    name=["y_" + "dd" + "_final"] * self.n_dimensions)
                
                

        for i, t in enumerate(np.linspace(0, 1, len(fx))):
            self.J += (y[0](t) - fx[i])**2 +  (y[1](t) - fy[i])**2

        try:
            self.J += 1e-4 * definite_integral(y[0].derivative().derivative()**2, 0, 1)
            self.J += 1e-4 * definite_integral(y[1].derivative().derivative()**2, 0, 1) 
        except:
            # print("Warning - Perhaps no second derivative?")
            pass
        
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g)
                }
        options = {'print_time': False, 'ipopt': {'print_level' : 0, 'max_iter': 1000, 'max_cpu_time': 100}}
        solver = nlpsol('solver', 'ipopt', prob, options)

        arg = {'lbx': self.lbw,
               'ubx': self.ubw,
               'lbg': self.lbg,
               'ubg': self.ubg
               }

        self.solution = solver.call(arg)

        basis = y[0].basis
        coeffs1 = self.solution['x'].full()[:len(basis)]
        coeffs2 = self.solution['x'].full()[len(basis):len(basis) * 2]

        fitted_spline = [BSpline(basis, coeffs1), BSpline(basis, coeffs2)]


        return fitted_spline




# fp = FrenetPath(-5, 5, 100)
# t = np.linspace(0, 1, 100)
# f = np.array([fp.t_to_xy(t_) for t_ in t])
# fx, fy = f[:, 0], f[:, 1]

# sf = SplineFitter()
# fitted_spline = sf.fitting(fx, fy)
# plt.figure()
# plt.plot(fx, fy)

# We need to fit the curve onto a spline...
# And the curvature too...

# Let's create a spline with unknown coefficients

# fx2, fy2 = [fitted_spline[0](t_)[0] for t_ in t],[fitted_spline[1](t_)[0] for t_ in t]
# plt.plot(fx2, fy2)

# from spline_extra import definite_integral
# definite_integral((1 - fitted_spline[0].derivative())**0.5, 0, 1)
