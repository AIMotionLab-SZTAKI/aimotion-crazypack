from .spline import BSpline, BSplineBasis
# from gurobipy import Model
# from gurobi import GRB
import numpy as np
import math



"Spline related functions"
def define_knots(degree = 3, **kwargs):
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

def define_gurobi_spline(model, degree, knot_intervals, n_spl, lower_bound, upper_bound, initial_values):
    """This function defines a set of splines.
    Input:
        basis: the basis function to define the spline with. If not provided,
        self.basis will be used.
        n_spl: number of splines to define
    Returns:
        a B-spline class
    """
    
    from gurobipy import Model
    basis = define_knots(degree = degree, knot_intervals = knot_intervals)

    splines = []
    for k in range(n_spl):
        # coeffs = MX.sym(name[k], len(basis))
        coeffs = model.addVars(len(basis), lb = lower_bound, ub = upper_bound)
        if len(initial_values) > 0:
            for i in range(len(initial_values[k])):
                coeffs[i].start = initial_values[k][i]
        splines += [BSpline(basis, np.array(coeffs.values()))]

    # print("Starting GUROBI optimization")
    # print("Number of variables: " + str(model.NumVars))
    # print("Number of constraints: " + str(model.NumConstrs))
        
        
    return np.array(splines), model

def define_constraint(model, constraint, lower_bound, upper_bound, constraint_type = 'overall', name = ''):
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
                model.addConstr((  constraint[i].coeffs[j] >= lower_bound[i]  ), 'overall_' + str(int(i)) if name == '' else name[i])
                model.addConstr((  constraint[i].coeffs[j] <= upper_bound[i]  ), 'overall_' + str(int(i)) if name == '' else name[i])
        return model

    # elif constraint_type == 'time':
    #     for i in range(len(constraint)):
    #         # for j in range(constraint[i].coeffs.shape[0]):
    #         self.g += [constraint[i]]
    #         for j in range(constraint[i].shape[0]):
    #             self.g_list += [name]
    #             self.lbg += [lower_bound[i]]
    #             self.ubg += [upper_bound[i]]
    #     return self

    elif constraint_type == 'initial':
        for i in range(len(constraint)):
            model.addConstr((  constraint[i].coeffs[0] >= lower_bound[i]  ), 'initial_' + str(int(i)) if name == '' else name[i])
            model.addConstr((  constraint[i].coeffs[0] <= upper_bound[i]  ), 'initial_' + str(int(i)) if name == '' else name[i])
        return model

    elif constraint_type == 'final':
        for i in range(len(constraint)):
            model.addConstr((  constraint[i].coeffs[-1] >= lower_bound[i]  ), 'final_' + str(int(i)) if name == '' else name[i])
            model.addConstr((  constraint[i].coeffs[-1] <= upper_bound[i]  ), 'final_' + str(int(i)) if name == '' else name[i])
        return model

    else:
        raise NotImplementedError()
        
def collision_avoidance_circular_gurobi(model, splines, center, radious, name = ''):
        """This function defines constraints on the splines to avoid the space arodund
        a certaint point with a given radious.
        Input:
            splines (list): a spline on which we want to set final constraint
            center: the point to avoid
            radious: the minimum distance from the center point
        Returns:

        """

        constraint = (splines[0] - center[0])**2 + (splines[1] - center[1])**2
        model = define_constraint(model, [constraint], lower_bound = [radious**2], upper_bound = [math.inf, math.inf], name = name)
        
        return model
        
        
        
        