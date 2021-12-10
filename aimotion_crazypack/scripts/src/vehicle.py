import numpy as np
import math
from .param import ParamValX, ParamValZ, DecisionVarX, DecisionVarZ
# from casadi import *
from casadi import MX, SX, Function, vertcat, dot, nlpsol, cos, sin, norm_2
from .frenet_path import FrenetPath
from .frenet_spline import SplineFitter
from numpy import interp



from .spline import BSpline, BSplineBasis
from .spline_extra import definite_integral
from .environment import Environment
from .astar import Node, return_path, astar

from collections import OrderedDict
from matplotlib.patches import Polygon
import matplotlib.patches as patches
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import csv
import time
import pickle
import random


class Vehicle(Environment):
    def __init__(self):
        super().__init__()
        self.stage = []
        self.n_dimensions = 2
        self.state_len = self.n_dimensions
        self.radious = 0.08
        self.T = 5
        self.cwd = ''

        self.x0 = []
        self.xf = []
        self.neighbours = []
        self.obstacles = []
        self.ID = -1

        # Temporary containers
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []

        self.P0_assemble = []
        self.P0_z_assemble = []

        self.initial_values = {}
        self.astar_initials = {}
        self.heuristic_initials = {}
        self.maze_printable = []

        # Test hyperparam
        self.slack = 0.001
        self.slack = 0.00001
        # self.slack = 0.0
        self.t_resolution_length = 30
        # Hyperparams
        self.rho = 50
        self.rho_formation = 100
        self.rho_input = 1

        self.epsilon = 0.01 # try to keep minimum epsilon distance from the obstacle
        self.safety_weight = 0 # cost parameter for epsilon
        self.knot_intervals = 15 # number of knots for the output (position) spline of the vehicle
        self.gurobi_time = 1.5 * 2 * 10
        self.gurobi_threads = 1
        self.state_degree = 3
        
        # FRENET
        self.frenet_enabled = False
        self.N = 100
        
        # These 2 values below will define the resolution of the heat-map.
        self.value_function_x_resolution = 50
        self.value_function_y_resolution = 20
        # self.value_function_x_resolution = 150
        # self.value_function_y_resolution = 50
        # self.value_function_x_resolution = 30
        # self.value_function_y_resolution = 10
        # self.value_function_x_resolution = 5
        # self.value_function_y_resolution = 3
        # self.value_function_x_resolution = 200
        # self.value_function_y_resolution = 100

        # Constraints on decision variables
        # self.y_min = [-20, -20]
        # self.y_max = [20, 20]
        self.y_min = [self.border_x[0], self.border_y[0]]
        self.y_max = [self.border_x[1], self.border_y[1]]

        # self.u_min = [-math.inf, -math.inf]
        # self.u_max = [math.inf, math.inf]
        self.u_min = [-50, -50]
        self.u_max = [50, 50]
        self.u_min = [-250, -250]
        self.u_max = [250, 250]

        # message
        self.message_in = {}
        self.message_out = {}


        # variable_history
        # We will store here the folded version of the data
        self.variable_history =  {'y' : [],         # x_update
                                  'y_j' : [],       # data_exchange_x_receive
        }
        self.solver_stats = []
        
        self.options = {'print_time': False, 'ipopt': {'print_level' : 0, 'max_iter': 1000, 'max_cpu_time': 100}}
        self.options_z = {'print_time': False, 'ipopt': {'print_level' : 0, 'max_iter': 1000, 'max_cpu_time': 100}}
        
        self.x_update_time = []
        self.z_update_time = []
        
    # def gurobi_spline_path(self, indices, x0, fig = [], axs = []):
    def gurobi_spline_path(self, inputs):
        """ This function generates a spline path from the input x0 state to the 
        self.xf final state.
        It reaturns the objective value when the trajectory is found and pickles the
        objective into a the folder 'data/vehicle_x/gurobi_value_function/' and file
        named '[number].pickle'
        """
        
        self.u_min = [-250, -250]
        self.u_max = [250, 250]
        
        path = inputs['path']
        count = inputs['count']
        indices = inputs['indices']
        x0 = inputs['x0']
        fig = inputs['fig']
        axs = inputs['axs']
        
        from gurobipy import Model
        from gurobi import GRB
        from .gurobi_spline import define_gurobi_spline, collision_avoidance_circular_gurobi, define_constraint
        model = Model("ppl")
        
        try:
            path = self.cwd + '/data/vehicle_0/astar_coeffs/'
            with open(path + str(int(count)) + '.pickle', 'rb') as f:
                coeffs = pickle.load(f)
        except:
            coeffs = []
            coeffs += [[0] * len(self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals))]
            coeffs += [[0] * len(self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals))]
            
        # Setting up decision variables
        x, model = define_gurobi_spline(model = model, degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = 1, lower_bound = self.border_x[0], upper_bound = self.border_x[1], initial_values = [coeffs[0]])
        y, model = define_gurobi_spline(model = model, degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = 1, lower_bound = self.border_y[0], upper_bound = self.border_y[1], initial_values = [coeffs[1]])
        
        x = x[0]
        y = y[0]
        
        vx = x.derivative()
        vy = y.derivative()
        ax = vx.derivative()
        ay = vy.derivative()
        
        # Defining constraints
        # x, y
        model = define_constraint(model = model, constraint = [x, y], lower_bound = x0, upper_bound = x0, constraint_type = 'initial', name = ['x_initial', 'y_initial'])
        # model = define_constraint(model = model, constraint = [x, y], lower_bound = self.x0, upper_bound = self.x0, constraint_type = 'initial', name = '')
        model = define_constraint(model = model, constraint = [x, y], lower_bound = self.xf, upper_bound = self.xf, constraint_type = 'final', name = ['x_final', 'y_final'])
        model = define_constraint(model = model, constraint = [x, y], lower_bound = [self.border_x[0], self.border_y[0]], upper_bound = [self.border_x[1], self.border_y[1]], constraint_type = 'overall', name = '')
        
        # vx, vy
        model = define_constraint(model = model, constraint = [vx, vy], lower_bound = [0, 0], upper_bound = [0, 0], constraint_type = 'initial', name = ['vx_initial', 'vy_initial'])
        model = define_constraint(model = model, constraint = [vx, vy], lower_bound = [0, 0], upper_bound = [0, 0], constraint_type = 'final', name = ['vx_final', 'vy_final'])
        # model = define_constraint(model = model, constraint = [vx, vy], lower_bound = self.v_min, upper_bound = self.v_max, constraint_type = 'overall', name = '')
        
        # ax, ay
        model = define_constraint(model = model, constraint = [ax, ay], lower_bound = [0, 0], upper_bound = [0, 0], constraint_type = 'initial', name = ['ax_initial', 'ay_initial'])
        model = define_constraint(model = model, constraint = [ax, ay], lower_bound = [0, 0], upper_bound = [0, 0], constraint_type = 'final', name = ['ax_final', 'ay_final'])
        model = define_constraint(model = model, constraint = [ax, ay], lower_bound = self.u_min, upper_bound = self.u_max, constraint_type = 'overall', name = '')
        
        # Collision avoidance
        R = 1e5
        from gurobi import GRB
        binary_collision_avoidance = True
        
        for obs_num, obs in enumerate(self.obstacles):
            center = obs.center
            radious = obs.size/2
            d_obs = radious
            d_obs = 0.01
            # d_obs = 0
            dx, dy = obs.dx, obs.dy
                
            if binary_collision_avoidance:
                # t = np.linspace(0, 1, self.knot_intervals * 2)
                
                # Sasmpling-based collision avoidance
                t = np.linspace(0, 1, 100)
                c = model.addVars(4, len(t), lb = 0, vtype = GRB.BINARY)
                
                # Initializing c
                # current guess for x:
                
                basis_x = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)    
                x_initial = BSpline(basis_x, coeffs[0])
                y_initial = BSpline(basis_x, coeffs[1])
                
                eq1 = [ x_initial(t_)[0] - (center[0] + dx) >=  d_obs for i, t_ in enumerate(t)]
                eq2 = [-x_initial(t_)[0] + (center[0] - dx) >=  d_obs for i, t_ in enumerate(t)]
                eq3 = [ y_initial(t_)[0] - (center[1] + dy) >=  d_obs for i, t_ in enumerate(t)]
                eq4 = [-y_initial(t_)[0] + (center[1] - dy) >=  d_obs for i, t_ in enumerate(t)]
                
                c1 = [0 if eq == True else 1 for eq in eq1]
                c0 = [0 if eq == True else 1 for eq in eq2]
                c3 = [0 if eq == True else 1 for eq in eq3]
                c2 = [0 if eq == True else 1 for eq in eq4]
                
                for i in range(len(c1)):
                    c[1, i].start = c1[i]
                    c[0, i].start = c0[i]
                    c[3, i].start = c3[i]
                    c[2, i].start = c2[i]
                    
                
                model.addConstrs((  x(t_)[0] - (center[0] + dx) >=  d_obs - R * c[1, i] for i, t_ in enumerate(t)  ), 'c[1,?]_obst_' + str(obs_num) + "_? = " )
                model.addConstrs(( -x(t_)[0] + (center[0] - dx) >=  d_obs - R * c[0, i] for i, t_ in enumerate(t)  ), 'c[0,?]_obst_' + str(obs_num) + "_? = " )
                model.addConstrs((  y(t_)[0] - (center[1] + dy) >=  d_obs - R * c[3, i] for i, t_ in enumerate(t)  ), 'c[3,?]_obst_' + str(obs_num) + "_? = " )
                model.addConstrs(( -y(t_)[0] + (center[1] - dy) >=  d_obs - R * c[2, i] for i, t_ in enumerate(t)  ), 'c[2,?]_obst_' + str(obs_num) + "_? = " )
            
                model.addConstrs((   c[0, i] + c[1, i] + c[2, i] + c[3, i] <= 3  for i, t_ in enumerate(t) ), 'c_SUM_obst_' + str(obs_num) + "_? = " )
            
            
                # Hyperplane collision avoidance
                # c = model.addVars(4, len(x.coeffs), lb = 0, vtype = GRB.BINARY)
                
                # model.addConstrs((   x.coeffs[i] * obs.a[0][0] + y.coeffs[i] * obs.a[0][1] >= 1*(obs.b[0] + self.radious*1 - R * c[0, i]) for i in range(len(x.coeffs))  ))
                # model.addConstrs((   x.coeffs[i] * obs.a[1][0] + y.coeffs[i] * obs.a[1][1] >= 1*(obs.b[1] + self.radious*1 - R * c[1, i]) for i in range(len(x.coeffs))  ))
                # model.addConstrs((   x.coeffs[i] * obs.a[2][0] + y.coeffs[i] * obs.a[2][1] >= 1*(obs.b[2] + self.radious*1 - R * c[2, i]) for i in range(len(x.coeffs))  ))
                # model.addConstrs((   x.coeffs[i] * obs.a[3][0] + y.coeffs[i] * obs.a[3][1] >= 1*(obs.b[3] + self.radious*1 - R * c[3, i]) for i in range(len(x.coeffs))  ))
                
                # model.addConstrs((   c[0, i] + c[1, i] + c[2, i] + c[3, i] <= 3  for i in range(len(x.coeffs))    ))
                
            
            else:
                # model = collision_avoidance_circular_gurobi(model = model, splines = [x, y], center = center, radious = radious)
                # model.setParam("NonConvex", 2)
                NotImplementedError()
        
        # Cost function
        J = 0
        J += definite_integral(self.rho * (ax)**2, 0, 1)   
        J += definite_integral(self.rho * (ay)**2, 0, 1)  
        J += definite_integral(self.rho / 100 * (x - self.xf[0])**2, 0, 1)  
        J += definite_integral(self.rho / 100 * (y - self.xf[1])**2, 0, 1)  
        
        # Start the optimization
        model.setObjective(J, GRB.MINIMIZE)
        model.Params.Threads = self.gurobi_threads
        model.Params.TimeLimit = self.gurobi_time
        
        t1 = time.time()
        model.optimize()
        t2 = time.time()
        
        print("gurobi time:" + str(t2-t1))
        print("Model status: ")
        print(model.status)
        if model.status == 3:
            objective = {}
            objective['ObjBound'] = -10
            objective['ObjBoundC'] = -10
            objective['ObjVal'] = -10
            return self, objective
        try:
            for i in range(len(x.coeffs)):
                tmp = x.coeffs[i].x
                tmp = y.coeffs[i].x
        except:
            objective = {}
            objective['ObjBound'] = -10
            objective['ObjBoundC'] = -10
            objective['ObjVal'] = -10
            return self, objective
        # Getting the solution
        sol = model.getVars()
        
        
        
        # x, y
        for i in range(len(x.coeffs)):
            x.coeffs[i] = x.coeffs[i].x
            y.coeffs[i] = y.coeffs[i].x
        
        
        # Plotting    
        t = np.linspace(0, 1, 100)
        x_t = [x(t_) for t_ in t]
        y_t = [y(t_) for t_ in t]
        
        
        if axs == []:
            fig, axs = plt.subplots()
        axs.plot(x_t, y_t)
        # axs.plot(x_t, y_t, '.')
        
        # Plotting the knot points
        # x_coeffs_t = [x(t_) for t_ in x.basis.knots[2:-2]]
        # y_coeffs_t = [y(t_) for t_ in y.basis.knots[2:-2]]
        # axs.plot(x_coeffs_t, y_coeffs_t, 'bo')
        
        
        # Plotting of obstacle
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(axs)
        # Plotting the environment
        axs = self.plot_environment(axs)
            
        plt.show()
        
        objective = {}
        objective['ObjBound'] = model.ObjBound
        objective['ObjBoundC'] = model.ObjBoundC
        objective['ObjVal'] = model.ObjVal
        
        
        # with open(path + '/' + str(count) + '.pickle', 'wb') as f:
        #     pickle.dump(objective, f)
        path = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/gurobi_value_function'
        with open(path + '/' + str(count) + '.pickle', 'wb') as f:
            pickle.dump({'objective': objective, 'indices': indices}, f)
            
            
        path = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/gurobi_coeffs'
        with open(path + '/' + str(count) + '.pickle', 'wb') as f:
            pickle.dump([[x.coeffs], [y.coeffs]], f)
            
            
            
        return self, objective
       
    def command_list_generation(self):
        """ This function creates a list, called environment. This environment
        is a matrix, whose values hold the objective for that given point in space.
        """
        fig, axs = plt.subplots()
        
        # Step 1: Creating a matrix of coordinates
        y = np.linspace(self.border_y[1], self.border_y[0], self.value_function_y_resolution + 2)
        x = np.linspace(self.border_x[0], self.border_x[1], self.value_function_x_resolution + 2)
        x = x[1:-1] # no points on the border
        y = y[1:-1] # no points on the border
        objective = {}
        objective['ObjBound'] = 0
        objective['ObjBoundC'] = 0
        objective['ObjVal'] = 0
        
        environment = []
        for i in range(len(y)):
            tmp = []
            for j in range(len(x)):
                tmp += [[x[j], y[i], objective]]
            environment.append(tmp)
            # environment.append([ [x[j], y[i], cost] for j in range(len(x)) ])
        
        # Step 2: adding obstacles to the environment
        for obs in self.obstacles:
            environment = obs.environment_space(environment)
        
        """ --- Plotting ---
        # Step 3: plotting the environment
        # black dots are the points, from where we will start an doptimization procedure
        # red dots are inside an obstacle, therefore no optimization will start from those
        for i in range(len(environment)):
            for j in range(len(environment[0])):
                if environment[i][j][2]['ObjVal'] < 0:
                    axs.plot(environment[i][j][0], environment[i][j][1], 'ro')
                else:
                    axs.plot(environment[i][j][0], environment[i][j][1], 'k.')
                    
                    
                    
        # Plotting of obstacle
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(axs)
        # Plotting the environment
        axs = self.plot_environment(axs)
        """
        
        
        
        # Step 4: Evaluating at each grid-point
        # for i in range(len(environment)):
        #     for j in range(len(environment[0])):
        gurobi_command_list = []
        indices_list = []
        count = 0
        for i in range(self.value_function_y_resolution):
            for j in range(self.value_function_x_resolution):
                indices = [i, j]
                x0 = [x[j], y[i]]
                inputs = {}
                inputs['path'] = ''
                inputs['count'] = count
                count += 1
                inputs['indices'] = indices
                inputs['x0'] = [x[j], y[i]]
                inputs['fig'] = []
                inputs['axs'] = []
                # self, objective = self.gurobi_spline_path(indices = [i, j], x0 = [x[j], y[i]], fig = fig, axs = axs)
                # self, objective = self.gurobi_spline_path(inputs)
                # environment[i][j][2] = objective
                
                
                # We don't add it to the command list, if it is inside an obstacle
                if environment[i][j][2]['ObjBound'] != -100:
                    gurobi_command_list += [inputs]
                    indices_list += [indices]
                
        # Saving the environment
        with open('data/environment.pickle', 'wb') as f:
            pickle.dump(environment, f)
        
        # Saving the command list, in case we want to use it later
        with open('data/gurobi_command_list.pickle', 'wb') as f:
            pickle.dump(gurobi_command_list, f)
            
        # # Saving the indices list, in case we want to use it later
        # with open('data/indices_list.pickle', 'wb') as f:
        #     pickle.dump(indices_list, f)
            
        self.environment = environment
         
        
        return self
    
    def value_function_generation(self):
        
        # Loading the environment, just in case we crash
        with open('data/environment.pickle', 'rb') as f:
            environment = pickle.load(f)
        
        # Loading the command list, in case we want to use it later
        with open('data/gurobi_command_list.pickle', 'rb') as f:
            gurobi_command_list = pickle.load(f)
            
            
        # # Loading the indices list, in case we want to use it later
        # with open('data/indices_list.pickle', 'rb') as f:
        #     indices_list = pickle.load(f)
            
        # k = 0
        # for i, j in indices_list:
        for k in range(len(gurobi_command_list)):
            inputs = gurobi_command_list[k]
            # k += 1
            self, objective = self.gurobi_spline_path(inputs)
            self, objective = self.astar_spline_path(inputs)
            # environment[i][j][2] = objective
            
            
        # Plotting the heat map    
        self.heat_map()  
        return True
        
    def value_function_trajectories(self):
        import os
        def list_text_files(path):
            """List all text files below a root directory."""
            for dirpath, dirname, filenames in os.walk(path):
                for filename in filenames:
                    if filename.endswith('.pickle'):
                        yield os.path.join(dirpath, filename)
                        
                        
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        t = np.linspace(0, 1, 100)
        # Gurobi
        gurobi_x_coeffs = []
        gurobi_y_coeffs = []
        path = self.cwd + '/data/vehicle_0/gurobi_coeffs'
        for file in list_text_files(path):
            with open(file, 'rb') as f:
                pickle_file = pickle.load(f)
                gurobi_x_coeffs += [pickle_file[0]]
                gurobi_y_coeffs += [pickle_file[1]]
                
        gurobi_x = [BSpline(basis, gurobi_x_coeffs_[0].tolist()) for gurobi_x_coeffs_ in gurobi_x_coeffs]
        gurobi_y = [BSpline(basis, gurobi_y_coeffs_[0].tolist()) for gurobi_y_coeffs_ in gurobi_y_coeffs]
        gurobi_x_t = [  [gurobi_x_(t_)[0] for t_ in t] for gurobi_x_ in gurobi_x]
        gurobi_y_t = [  [gurobi_y_(t_)[0] for t_ in t] for gurobi_y_ in gurobi_y]
        
        # A-star
        astar_x_coeffs = []
        astar_y_coeffs = []
        path = self.cwd + '/data/vehicle_0/astar_coeffs'
        for file in list_text_files(path):
            with open(file, 'rb') as f:
                pickle_file = pickle.load(f)
                astar_x_coeffs += [pickle_file[0]]
                astar_y_coeffs += [pickle_file[1]]
        
        astar_x = [BSpline(basis, astar_x_coeffs_) for astar_x_coeffs_ in astar_x_coeffs]
        astar_y = [BSpline(basis, astar_y_coeffs_) for astar_y_coeffs_ in astar_y_coeffs]
        astar_x_t = [  [astar_x_(t_)[0] for t_ in t] for astar_x_ in astar_x]
        astar_y_t = [  [astar_y_(t_)[0] for t_ in t] for astar_y_ in astar_y]
        
        # A-star path
        astar_x_path = []
        astar_y_path  = []
        path = self.cwd + '/data/vehicle_0/astar_path'
        for file in list_text_files(path):
            with open(file, 'rb') as f:
                pickle_file = pickle.load(f)
                astar_x_path  += [pickle_file[0]]
                astar_y_path  += [pickle_file[1]]
        
        fig, axs = plt.subplots()
        # Plotting the environment
        axs = self.plot_environment(axs)
        # Plotting of obstacles
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(axs)
            
        """ ::)) """
        # Step 1: Creating a matrix of coordinates
        y = np.linspace(self.border_y[1], self.border_y[0], self.value_function_y_resolution + 2)
        x = np.linspace(self.border_x[0], self.border_x[1], self.value_function_x_resolution + 2)
        x = x[1:-1] # no points on the border
        y = y[1:-1] # no points on the border
        objective = {}
        objective['ObjBound'] = 0
        objective['ObjBoundC'] = 0
        objective['ObjVal'] = 0
        environment = []
        for i in range(len(y)):
            tmp = []
            for j in range(len(x)):
                tmp += [[x[j], y[i], objective]]
            environment.append(tmp)
            # environment.append([ [x[j], y[i], cost] for j in range(len(x)) ])
        
        # Step 2: adding obstacles to the environment
        for obs in self.obstacles:
            environment = obs.environment_space(environment)
        # Step 3: plotting the environment
        # black dots are the points, from where we will start an doptimization procedure
        # red dots are inside an obstacle, therefore no optimization will start from those
        for i in range(len(environment)):
            for j in range(len(environment[0])):
                if environment[i][j][2]['ObjVal'] < 0:
                    axs.plot(environment[i][j][0], environment[i][j][1], 'ro')
                else:
                    axs.plot(environment[i][j][0], environment[i][j][1], 'k.')
        """ ::)) """            
                    
                    
        
        
        
        from matplotlib.pyplot import cm
        max_length = max(len(gurobi_x_t), len(astar_x_t), len(astar_x_path))    
        color=cm.Wistia(np.linspace(0,1,max_length))
        color=cm.YlOrRd(np.linspace(0,1,max_length))
        color=cm.hsv(np.linspace(0,1,max_length))
        # color=cm.YlOrRd(np.linspace(0,1,len(hist["y"])))
        
        for i in range(max_length):
            if len(gurobi_x_t) > i:
                axs.plot(gurobi_x_t[i], gurobi_y_t[i], c = color[i])
            if len(astar_x_t) > i:
                axs.plot(astar_x_t[i], astar_y_t[i], c = color[i], linestyle = 'dashed')
            if len(astar_x_path) > i:
                axs.plot(astar_x_path[i], astar_y_path[i], c = color[i], linestyle = 'dotted') 
            
        axs.set_title('Trajectories of the path generated by the astar algorithm (dotted), astar BSpline (dashed) and gurobi (full)')
        plt.savefig(self.cwd + "/figures/value_function_trajectories_vehicle"+ str(self.ID)+ ".eps")
        # for i in range(len(gurobi_x_t)):
        #     axs.plot(gurobi_x_t[i], gurobi_y_t[i])
            
        # for i in range(len(astar_x_t)):
        #     axs.plot(astar_x_t[i], astar_y_t[i], linestyle = 'dashed')
            
        # for i in range(len(astar_x_path)):
        #     axs.plot(astar_x_path[i], astar_y_path[i], linestyle = 'dotted')  
            
        
        plt.show()
        return self
    
    def heat_map(self, algorithm = 'astar'):
        """ This function creates the heat-map. It reads the file named environment.pickle
        and plots the objective function at each gridpoint.
        """
        
        # loading the appropriate module
        try:
            from mpl_toolkits import mplot3  # for some reasin this might not work
        except:
            from mpl_toolkits.mplot3d import Axes3D
            from mpl_toolkits.mplot3d import axes3d
        
        # Loading the file
        with open('data/environment.pickle', 'rb') as f:
            environment = pickle.load(f)
            
        # Loading the objectives
        import os
        
        def list_text_files(path):
            """List all text files below a root directory."""
            for dirpath, dirname, filenames in os.walk(path):
                for filename in filenames:
                    if filename.endswith('.pickle'):
                        yield os.path.join(dirpath, filename)
                        
        if algorithm == 'gurobi':
            # root = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/gurobi_value_function'
            path = self.cwd + '/data/vehicle_0/gurobi_value_function'
        elif algorithm == 'astar':
            # root = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/astar_value_function'
            path = self.cwd + '/data/vehicle_0/astar_value_function'
        else:
            NotImplementedError()
            
            
        objective_list = []
        indices_list = []
        for file in list_text_files(path):
            with open(file, 'rb') as f:
                pickle_file = pickle.load(f)
                indices_list += [pickle_file['indices']]
                objective_list += [pickle_file['objective']]
            
            
        count = 0
        # for i in range(len(environment)):
        #     for j in range(len(environment[0])):
        #         if count < len(objective_list):
            
        max_objective = 0
        min_objective = math.inf
        for indice in indices_list:
            i, j = indice
            environment[i][j][2] = objective_list[count]
            if objective_list[count]['ObjVal'] > max_objective:
                max_objective = objective_list[count]['ObjVal']
            if objective_list[count]['ObjVal'] < min_objective and objective_list[count]['ObjVal'] > 0:
                min_objective = objective_list[count]['ObjVal']
                
            count += 1
                
        max_objective = max_objective + (max_objective - min_objective) * 1e-1  
        
        # Plotting
        x, y, z = [], [], []
        x_obs, y_obs, z_obs = [], [], []
        x_infs, y_infs, z_infs = [], [], []
        x_tri, y_tri, z_tri = [], [], []
        for j in range(len(environment)):
            for i in range(len(environment[0])):
                # For the 1st plot (scatter)
                x += [environment[j][i][0]]
                y += [environment[j][i][1]]
                z += [environment[j][i][2]['ObjVal']]
                
                # For the 2nd plot (trisurf)
                # The coordinates, which are inside obstacles are marked with a value of -100
                # try:
                #     environment[j][i][2]['ObjVal'] == -100
                # except:
                #     kappa = True
                    
                # print(environment[j][i][2]['ObjVal'])
                if environment[j][i][2]['ObjVal'] == -100:
                # if environment[j][i][2]['ObjVal'] == max_objective * 10:
                    x_obs += [environment[j][i][0]]
                    y_obs += [environment[j][i][1]]
                    # z_obs += [environment[j][i][2]['ObjVal']]
                    z_obs += [max_objective]
                # The coordinates, from which trajectories coiuld not be generated are marked with a value of -10
                # TODO: do something, where ObjVal == 0. I don't know why we have that... (in cases, when we have actaully explored that point)
                elif environment[j][i][2]['ObjVal'] == -10 or environment[j][i][2]['ObjVal'] == 0:
                # elif environment[j][i][2]['ObjVal'] == max_objective * 10:
                    x_infs += [environment[j][i][0]]
                    y_infs += [environment[j][i][1]]
                    # z_infs += [environment[j][i][2]['ObjVal']]
                    z_infs += [max_objective]
                # Every other coordinate will be stored in the following variables:
                else:
                    x_tri += [environment[j][i][0]]
                    y_tri += [environment[j][i][1]]
                    z_tri += [environment[j][i][2]['ObjVal']]
                    
                    
        """
        sighh....
        """
        def linspace_filter(environment):
            # Step 1
            # for every row, we step through each column
            # if value < 100, we take the previous value and the next value (if these exists)
            # and linspace them
            
            # Step 2
            # we will do the same in the opposite direction, meaning for every column we step 
            # through every row
            
            # Step 1
            prev = []
            succ = []
            k = 0
            for i in range(len(environment)):
                for j in range(len(environment[0])):
                    current = environment[i][j][2]['ObjVal']
                    if current < 0:
                        # We search for a value in the succeeding value, that is not < 0
                        k = 1
                        while( environment[i][j + k][2]['ObjVal'] < 0 and j+k < len(environment[i]) ):
                            succ = environment[i][j + k][2]['ObjVal']
                            k += 1
                    if succ != []:
                        if prev != []:
                            environment[i][j] = np.linspace(prev, succ, k - 1)[1]
                        # No, ezt folytassuk majd szeptember 28 után
                        # TODO :)
                    
                    prev = current
                    succ = []
                prev = []
        
        
        def averaging_filter(environment):
            
            len_x = len(environment[0])
            len_y = len(environment)
            
            kernel_x = int(len_x / 10)
            kernel_y = int(len_y / 10)
            
            step_x0 = 0
            step_xf = step_x0 + kernel_x
            # idx = 
            
            
        return environment
        """
        sighh.... END
        """
        
        
                    
        # 1st plot: scatter          
        fig = plt.figure() 
        ax = plt.axes(projection='3d') 
        # shifting and takeing the log
        z = np.log(np.array(z) + 101)
        ax.scatter(x, y, z, c=z, cmap='viridis', linewidth=0.5);
        ax.set_title('Scatter plot of the heat_map generated by the algorithm: '  + algorithm)
        plt.savefig(self.cwd + "/figures/scatter_plot_" + algorithm + "_vehicle"+ str(self.ID)+ ".eps")
        
        # 2nd plot: trisurf
        fig = plt.figure() 
        ax = plt.axes(projection='3d')
        # shifting and taking the log
        # z_obs = np.log(np.array(z_obs) + 101) 
        # z_infs = np.log(np.array(z_infs) + 101)
        # z_tri = np.log(np.array(z_tri) + 101)
        ax.scatter(x_obs, y_obs, z_obs, c=z_obs, cmap='viridis', linewidth=0.5);
        ax.scatter(x_infs, y_infs, z_infs, c='r', linewidth=0.5);
        try:
            ax.plot_trisurf(x_tri, y_tri, z_tri, cmap='viridis', edgecolor='none');
        except:
            ax.scatter(x_tri, y_tri, z_tri, c='r', linewidth=0.5);
        ax.set_title('Trisurf plot of the heat_map generated by the algorithm: '  + algorithm)
        plt.savefig(self.cwd + "/figures/trisurf_plot_" + algorithm + "_vehicle"+ str(self.ID)+ ".eps")
        
        # :))
        x_linspace = np.linspace(environment[0][0][0], environment[0][-1][0], 100)
        y_linspace = np.linspace(environment[0][0][1], environment[-1][0][1], 100)
        z_linspace = []
        for i in range(100):
            z_linspace += [ax.format_coord(x_linspace[i], y_linspace[i])]
            
        from scipy.spatial import Delaunay
        points2D = np.vstack([x_tri,y_tri]).T    
        tri = Delaunay(points2D)
        simplices = tri.simplices
        
        # 3rd plot: trisurf, but x_obs is not separated
        fig = plt.figure() 
        ax = plt.axes(projection='3d')
        ax.plot_trisurf(x_tri+x_obs, y_tri+y_obs, z_tri+z_obs, cmap='viridis', edgecolor='none');
        ax.set_title('Combined trisurf plot of the heat_map generated by the algorithm: '  + algorithm)
        plt.savefig(self.cwd + "/figures/combined_trisurf_plot_" + algorithm + "_vehicle"+ str(self.ID)+ ".eps")
        
        
        return self
        
    def gurobi_path(self, fig = [], axs = []):
        """ This function generates a path from self.x0 to self.xf for the vehicle.
        This is not the spline version though ;)
        """
        v_min, v_max = -0.225, 0.225
        v_min, v_max = -10, 10
        v_min, v_max = -math.inf, math.inf
        
        
        T = 1
        dt = 0.06
        dt = 0.01
        N = int(T/dt)
        
        # from gurobipy import *
        from gurobipy import Model
        # N = 100
        model = Model("ppl")
        # model.Params.LogToConsole = 0
        # v_min, v_max = -100, 100
        # T = 1
        # dt = T / N
        M = 75 # number of constraints in order to approximate the force and velocity magnitudes
        
        
        # Setting up decision variables
        x = model.addVars(N, lb = self.border_x[0], ub = self.border_x[1])
        y = model.addVars(N, lb = self.border_y[0], ub = self.border_y[1])
        
        vx = model.addVars(N, lb = v_min, ub = v_max)
        vy = model.addVars(N, lb = v_min, ub = v_max)
        
        ax = model.addVars(N, lb = self.u_min[0], ub = self.u_max[0])
        ay = model.addVars(N, lb = self.u_min[1], ub = self.u_max[1])
        
        # Dynamics constraints
        model.addConstrs((  x[i+1] == x[i] + dt * vx[i] for i in range(N-1)  ))
        model.addConstrs((  y[i+1] == y[i] + dt * vy[i] for i in range(N-1)  ))
        model.addConstrs((  vx[i+1] == vx[i] + dt * ax[i] for i in range(N-1)  ))
        model.addConstrs((  vy[i+1] == vy[i] + dt * ay[i] for i in range(N-1)  ))
        
        # Initial constraints
        model.addConstr(x[0] == self.x0[0])
        model.addConstr(y[0] == self.x0[1])
        model.addConstr(vx[0] == 0)
        model.addConstr(vy[0] == 0)
        # model.addConstr(ax[0] == 0)
        # model.addConstr(ay[0] == 0)
        
        # Final constraints
        # model.addConstr(x[N-1] == self.xf[0])
        # model.addConstr(y[N-1] == self.xf[1])
        # model.addConstr(vx[N-1] == 0)
        # model.addConstr(vy[N-1] == 0)
        # model.addConstr(ax[N-1] == 0)
        # model.addConstr(ay[N-1] == 0)
        
        # Maximum velocity constraints
        # TODO: is this, how it is supposed to be?
        # for m_small in range(1, M+1):
        #     model.addConstrs((  vx[i] * np.cos(2*np.pi * m_small / M) \
        #                      + vy[i] * np.sin(2*np.pi * m_small / M)  \
        #                      <= np.sqrt(v_max**2 + v_max**2)    \
        #                      for i in range(N) \
        #                             ))
        # # Maximum acceleration constraints
        # # TODO: is this, how it is supposed to be?
        # for m_small in range(1, M+1):
        #     model.addConstrs((  ax[i] * np.cos(2*np.pi * m_small / M) \
        #                      + ay[i] * np.sin(2*np.pi * m_small / M)  \
        #                      <= np.sqrt(self.u_max[0]**2 + self.u_max[1]**2)    \
        #                      for i in range(N) \
        #                             ))
        
        model.addConstrs((  vx[i] >= v_min for i in range(N)  ))
        model.addConstrs((  vx[i] <= v_max for i in range(N)  ))
        model.addConstrs((  vy[i] >= v_min for i in range(N)  ))
        model.addConstrs((  vy[i] <= v_max for i in range(N)  ))
        # model.addConstrs((  ax[i] >= self.u_min[0] for i in range(N)  ))
        # model.addConstrs((  ax[i] <= self.u_max[0] for i in range(N)  ))
        # model.addConstrs((  ay[i] >= self.u_min[1] for i in range(N)  ))
        # model.addConstrs((  ay[i] <= self.u_max[1] for i in range(N)  ))
                
        # TODO: missing the fm part
        
        
        # Final position constraints
        R = 1e5
        from gurobi import GRB
        b = model.addVars(N, lb = 0, vtype = GRB.BINARY)
        
        for i in range(N):
            model.addConstr(x[i] - self.xf[0] <= R*(1 - b[i]))
            model.addConstr(x[i] - self.xf[0] >=-R*(1 - b[i]))
            model.addConstr(y[i] - self.xf[1] <= R*(1 - b[i]))
            model.addConstr(y[i] - self.xf[1] >=-R*(1 - b[i]))
            model.addConstr(vx[i] - 0 <= R*(1 - b[i]))
            model.addConstr(vx[i] - 0 >=-R*(1 - b[i]))
            model.addConstr(vy[i] - 0 <= R*(1 - b[i]))
            model.addConstr(vy[i] - 0 >=-R*(1 - b[i]))
            
        model.addConstr(b.sum() == 1)
        
        # Collision avoidance constraints
        d_obs = 0.1
        d_obs = self.radious * 1
        for obs in self.obstacles:
            c = model.addVars(4, N, lb = 0, vtype = GRB.BINARY)
        
            model.addConstrs((  x[i] - (obs.center[0] + obs.dx) >=  d_obs - R * c[1, i] for i in range(N - 1)  ))
            model.addConstrs(( -x[i] + (obs.center[0] - obs.dx) >=  d_obs - R * c[0, i] for i in range(N - 1)  ))
            model.addConstrs((  y[i] - (obs.center[1] + obs.dy) >=  d_obs - R * c[3, i] for i in range(N - 1)  ))
            model.addConstrs(( -y[i] + (obs.center[1] - obs.dy) >=  d_obs - R * c[2, i] for i in range(N - 1)  ))
        
            model.addConstrs((   c[0, i] + c[1, i] + c[2, i] + c[3, i] <= 3 for i in range(N-1)   ))
            
            # obs.center[0], obs.center[1], obs.dx, obs.dy
        
        # Objective function
        J = 0
        epsilon = 0.01
        for i in range(N):
            J += b[i] * i  + ax[i]**2 * epsilon * i + ay[i]**2 * epsilon * i
            
        model.setObjective(J, GRB.MINIMIZE)
        model.Params.Threads = self.gurobi_threads
        model.Params.TimeLimit = self.gurobi_time
        model.Params.TimeLimit = 100
        
        t1 = time.time()
        model.optimize()
        t2 = time.time()
        print("gurobi time:" + str(t2-t1))
        
        sol = model.getVars()
        model.getVars()
        
        finish_index = [b[i].x for i in range(len(b))]
        finish_index = np.where(np.array(finish_index)==1.0)[0]
        
        N = int(finish_index) + 1
        sol_x = [x[i].x for i in range(N)]
        sol_y = [y[i].x for i in range(N)]
        
        self.gurobi_x = sol_x
        self.gurobi_y = sol_y
        
        sol_ax = [ax[i].x for i in range(N)]
        sol_ay = [ay[i].x for i in range(N)]
        
        if fig == []:
            fig, axs = plt.subplots()
        
        # Plotting of obstacle
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(axs)
        # Plotting the environment
        axs = self.plot_environment(axs)
        axs.plot(sol_x, sol_y)
        
        objective = {}
        objective['ObjBound'] = model.ObjBound
        objective['ObjBoundC'] = model.ObjBoundC
        objective['ObjVal'] = model.ObjVal
        return self, objective
        
    def astar_path_gurobi_warm(self, inputs):
        path = inputs['path']
        count = inputs['count']
        indices = inputs['indices']
        x0 = inputs['x0']
        x_grid, y_grid = x0
        fig = inputs['fig']
        axs = inputs['axs']
        # Getting obstacle positions in the maze
        maze = [[0] * self.value_function_x_resolution for i in range(self.value_function_y_resolution)]
        for i in range(len(self.obstacles)):
            maze = self.obstacles[i].grid_world(maze)

        # for i in [0, 2, 3]:
        #     maze = self.obstacles[i].grid_world(maze)


        # Getting our own position in the maze
        x_grid = np.linspace(self.border_x[0], self.border_x[1], self.value_function_x_resolution + 2).tolist()
        y_grid = np.linspace(self.border_y[1], self.border_y[0], self.value_function_y_resolution + 2).tolist()
        x_grid = x_grid[1:-1]
        y_grid = y_grid[1:-1]

        # x0 = self.x0
        xf = self.xf

        x0_index = 0
        xf_index = len(x_grid) - 1
        y0_index = 0
        yf_index = len(y_grid) - 1
        dist_x0 = (x0[0] - x_grid[x0_index])**2 + (x0[1] - y_grid[y0_index])**2
        dist_xf = (xf[0] - x_grid[xf_index])**2 + (xf[1] - y_grid[yf_index])**2

        for i, y in enumerate(y_grid):
            for j, x in enumerate(x_grid):
                # Placing x0 in the grid world
                dist_x0_ = (x0[0] - x_grid[j])**2 + (x0[1] - y_grid[i])**2
                if dist_x0_ < dist_x0:
                    dist_x0 = dist_x0_
                    x0_index = j
                    y0_index = i

                # Placing xf in the grid world
                dist_xf_ = (xf[0] - x_grid[j])**2 + (xf[1] - y_grid[i])**2
                if dist_xf_ < dist_xf:
                    dist_xf = dist_xf_
                    xf_index = j
                    yf_index = i


        # Just checking if the results are correct
        # print("x0_real:", x0[:2])
        # print("x0_grid:", [x_grid[x0_index], y_grid[y0_index]])
        # print("xf_real:", xf[:2])
        # print("xf_grid:", [x_grid[xf_index], y_grid[yf_index]])


        # # Printing the maze
        # maze[y0_index][x0_index] = 2
        # maze[yf_index][xf_index] = 2
        # for row in maze:
        #   line = []
        #   for col in row:
        #     if col == 1:
        #       line.append("\u2588")
        #     elif col == 0:
        #       line.append(".")
        #     elif col == 2:
        #       line.append("*")
        #   print("".join(line))


        maze[y0_index][x0_index] = 0
        maze[yf_index][xf_index] = 0


        # t1 = time.time()
        path = astar(maze, (y0_index, x0_index), (yf_index, xf_index))
        
        if path == None:
            n_of_steps = max(abs(x0_index - xf_index), abs(y0_index - yf_index))
            if n_of_steps == 0:
                n_of_steps += 1
            x = np.linspace(x0_index, xf_index, n_of_steps, dtype = int)
            y = np.linspace(y0_index, yf_index, n_of_steps, dtype = int)
            
            path = [[y_, x_] for y_, x_ in zip(y, x)]
        # t2 = time.time()
        # print("astar time:" + str(t2-t1))


        for step in path:
            maze[step[0]][step[1]] = 2

        # Printing the maze & the result

        maze[y0_index][x0_index] = 3
        maze[yf_index][xf_index] = 3
        for row in maze:
          line = []
          for col in row:
            if col == 1:
              line.append("\u2588")
            elif col == 0:
              line.append(" ")
            elif col == 2:
              line.append(".")
            elif col == 3:
              line.append("*")
          # print("".join(line))
          # self.maze_printable += ["".join(line)]


        
        
        fitter = SplineFitter(knot_intervals = self.knot_intervals)

        fx_astar = [x_grid[x] for y, x in path]
        fy_astar = [y_grid[y] for y, x in path]
        
        # Cheating: replacing the last element with xf, because it is usually not a grid-point
        try:
            fx_astar[-1] = self.xf[0]
            fy_astar[-1] = self.xf[1]
        except:
            kappa = True
        
        
        # self.astar_x = fx_astar
        # self.astar_y = fy_astar


        if axs == []:
            fig, axs = plt.subplots()
        axs.plot(fx_astar, fy_astar)
        
        
        # plt.plot(fx_astar, fy_astar)
        lbw_x = [-math.inf,-math.inf, self.u_min[0]]
        lbw_y = [-math.inf,-math.inf, self.u_min[1]]
        ubw_x = [ math.inf, math.inf, self.u_max[0]]
        ubw_y = [ math.inf, math.inf, self.u_max[1]]
        y_astar = fitter.fitting(fx_astar, fy_astar, lbw = [lbw_x, lbw_y], ubw = [ubw_x, ubw_y] )
        # y_astar = fitter.fitting(fx_astar, fy_astar)
        # self.astar_initials["y_astar"] = y_astar
        #
        # In this part we want to initialize a, b, d_tau manually.
        # As can be read in the description of the "collision_avoidance_hyperplane()"
        # function, a is a vector, pointing towards the obstacle.
        # What we will do is we take the vector, pointing towards the obstacle,
        # which will be our a.
        # We can do as follows: 1) vector towards the middle point of the obstacle
        # 2) calculate a vector for each side of the obstacle. Use the one, whose normal
        # doesn't intersect the obstacle.
        # We will implement 1) for now, hopefully this solution will be good enough.
        # b will be zero and d_tau will be self.epsilon

        # Sample from the spline y_astar. Important: len(fx_star) amount!

        # fx = [y_astar[0](t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, len(fx_astar))]
        # fy = [y_astar[1](t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, len(fy_astar))]
        fx = [y_astar[0](t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, 100)]
        fy = [y_astar[1](t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, 100)]
        # axs.plot(fx, fy, 'k.')
        axs.plot(fx, fy)
        plt.show()
        
        J = 0
        J += definite_integral(self.rho * (y_astar[0].derivative().derivative())**2, 0, 1)[0]  
        J += definite_integral(self.rho * (y_astar[1].derivative().derivative())**2, 0, 1)[0] 
        # J += definite_integral(self.rho * (y_astar[0])**2, 0, 1)[0]  
        # J += definite_integral(self.rho * (y_astar[1])**2, 0, 1)[0]  

        fx_np = np.array(fx_astar)
        fy_np = np.array(fy_astar)
        # J += np.sum(np.diff(fx_np)**2) + np.sum(np.diff(fy_np)**2)
        
        objective = {}
        objective['ObjBound'] = J
        objective['ObjBoundC'] = J
        objective['ObjVal'] = J
        
        
        # with open(path + '/' + str(count) + '.pickle', 'wb') as f:
        #     pickle.dump(objective, f)
        # path = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/astar_value_function'
        path = self.cwd + '/data/vehicle_0/astar_value_function'
        with open(path + '/' + str(count) + '.pickle', 'wb') as f:
            pickle.dump({'objective': objective, 'indices': indices}, f)
            
            
        # path = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/astar_coeffs'
        path = self.cwd + '/data/vehicle_0/astar_coeffs'
        with open(path + '/' + str(count) + '.pickle', 'wb') as f:
            coeffs =  [y_astar[0].coeffs.reshape(1, -1).tolist()[0]] + \
                        [y_astar[1].coeffs.reshape(1, -1).tolist()[0]]
            pickle.dump(coeffs, f)
            
        # path = '/Users/szilard/Dropbox/Sztaki/code/code_examples/bspline_static/data/vehicle_0/astar_path'
        path = self.cwd + '/data/vehicle_0/astar_path'
        with open(path + '/' + str(count) + '.pickle', 'wb') as f:
            coeffs =  [fx_astar] + \
                        [fy_astar]
            pickle.dump(coeffs, f)
        
        return self, objective
    
    def astar_path(self):
        resolution = 100
        # Getting obstacle positions in the maze
        maze = [[0] * resolution for i in range(resolution)]
        for i in range(len(self.obstacles)):
            maze = self.obstacles[i].grid_world(maze)

        # for i in [0, 2, 3]:
        #     maze = self.obstacles[i].grid_world(maze)


        # Getting our own position in the maze
        x_grid = np.linspace(self.border_x[0], self.border_x[1], resolution).tolist()
        y_grid = np.linspace(self.border_y[1], self.border_y[0], resolution).tolist()

        x0 = self.x0
        xf = self.xf

        x0_index = 0
        xf_index = len(x_grid) - 1
        y0_index = 0
        yf_index = len(y_grid) - 1
        dist_x0 = (x0[0] - x_grid[x0_index])**2 + (x0[1] - y_grid[y0_index])**2
        dist_xf = (xf[0] - x_grid[xf_index])**2 + (xf[1] - y_grid[yf_index])**2

        for i, y in enumerate(y_grid):
            for j, x in enumerate(x_grid):
                # Placing x0 in the grid world
                dist_x0_ = (x0[0] - x_grid[j])**2 + (x0[1] - y_grid[i])**2
                if dist_x0_ < dist_x0:
                    dist_x0 = dist_x0_
                    x0_index = j
                    y0_index = i

                # Placing xf in the grid world
                dist_xf_ = (xf[0] - x_grid[j])**2 + (xf[1] - y_grid[i])**2
                if dist_xf_ < dist_xf:
                    dist_xf = dist_xf_
                    xf_index = j
                    yf_index = i


        # Just checking if the results are correct
        # print("x0_real:", x0[:2])
        # print("x0_grid:", [x_grid[x0_index], y_grid[y0_index]])
        # print("xf_real:", xf[:2])
        # print("xf_grid:", [x_grid[xf_index], y_grid[yf_index]])


        # # Printing the maze
        # maze[y0_index][x0_index] = 2
        # maze[yf_index][xf_index] = 2
        # for row in maze:
        #   line = []
        #   for col in row:
        #     if col == 1:
        #       line.append("\u2588")
        #     elif col == 0:
        #       line.append(" ")
        #     elif col == 2:
        #       line.append(".")
        #   print("".join(line))


        maze[y0_index][x0_index] = 0
        maze[yf_index][xf_index] = 0




        t1 = time.time()
        path = astar(maze, (y0_index, x0_index), (yf_index, xf_index))
        t2 = time.time()
        print("astar time:" + str(t2-t1))


        for step in path:
            maze[step[0]][step[1]] = 2

        # Printing the maze & the result

        maze[y0_index][x0_index] = 3
        maze[yf_index][xf_index] = 3
        for row in maze:
          line = []
          for col in row:
            if col == 1:
              line.append("\u2588")
            elif col == 0:
              line.append(" ")
            elif col == 2:
              line.append(".")
            elif col == 3:
              line.append("*")
          # print("".join(line))
          self.maze_printable += ["".join(line)]



        fitter = SplineFitter(knot_intervals = self.knot_intervals)

        fx_astar = [x_grid[x] for y, x in path]
        fy_astar = [y_grid[y] for y, x in path]
        
        self.astar_x = fx_astar
        self.astar_y = fy_astar


        plt.plot(fx_astar, fy_astar)
        y_astar = fitter.fitting(fx_astar, fy_astar)
        self.astar_initials["y_astar"] = y_astar
        #
        # In this part we want to initialize a, b, d_tau manually.
        # As can be read in the description of the "collision_avoidance_hyperplane()"
        # function, a is a vector, pointing towards the obstacle.
        # What we will do is we take the vector, pointing towards the obstacle,
        # which will be our a.
        # We can do as follows: 1) vector towards the middle point of the obstacle
        # 2) calculate a vector for each side of the obstacle. Use the one, whose normal
        # doesn't intersect the obstacle.
        # We will implement 1) for now, hopefully this solution will be good enough.
        # b will be zero and d_tau will be self.epsilon

        # Sample from the spline y_astar. Important: len(fx_star) amount!

        fx = [y_astar[0](t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, len(fx_astar))]
        fy = [y_astar[1](t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, len(fy_astar))]
        plt.plot(fx, fy, 'k.')

        # Fill w0_coeffs with the coefficients of y_astar
        w0_coeffs = [] + y_astar[0].coeffs.reshape(1, -1).tolist()[0] \
            + y_astar[1].coeffs.reshape(1, -1).tolist()[0]

        for obstacle in self.obstacles:
            x_c, y_c = obstacle.center
            a_x = []
            a_y = []
            for i in range(len(fx_astar)):
                # a_x += [fx_astar[i] - fx[i]]
                # a_y += [fy_astar[i] - fy[i]]
                # What the hell is the above code and why was it working?
                # Anyhow...
                # This:
                a_x += [x_c - fx_astar[i]]
                a_y += [y_c - fy_astar[i]]
                # Or:
                # a_x += [x_c - fx[i]]
                # a_y += [y_c - fy[i]]

                # b += [0]
                # d_tau += [self.epsilon]

            # Now we want to fit a_x and a_y onto a spline, so that we can use its coefficients
            # to warm-start the optimizer. Important: a, b, d_tau are degree = 1 splines!
            # TODO: do we really need to do this fitting? Can't we just sample from it?
            a_spline = fitter.fitting(a_x, a_y, degree = 1)
            xy_component = [spline.coeffs.reshape(1, -1).tolist()[0] for spline in a_spline]
            a_coeffs = xy_component[0] + xy_component[1]
            b_coeffs = [0] * len(a_spline[0].basis)
            d_tau_coeffs = [self.epsilon] * len(a_spline[0].basis)
            # assembling w0_coeffs
            w0_coeffs += a_coeffs + b_coeffs + d_tau_coeffs


        # ------
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []
        # Okay. We have the path. Now we want find the corresponding a, b, d_tau values
        # But how do we do that?
        # Step 1: get y_astar
        # (we have done so above)

        # plt.savefig(self.cwd + "/figures/astar_vehicle_"+ str(self.ID)+ ".png")
        # Step 2: y shluld be close to y_astar

        # Step 3: apply constarints on y
        # Step 4: optimize
        # Step 5: recode def define_MX_spline() such that it accepts entire
        # initial values, not just their starting point and ending point.

        y = self.define_MX_spline(degree=self.state_degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                initial_value = [y_astar[0].coeffs.reshape(-1,).tolist(), y_astar[0].coeffs.reshape(-1,).tolist()],
                                name=["y"] * self.n_dimensions)

        # Initial position constraint on y
        self.define_constraint(y,
                               self.x0[:self.n_dimensions],
                               self.x0[:self.n_dimensions],
                               constraint_type='initial',
                               name=["y0"] * self.n_dimensions)


        # Final position constraint on y
        self.define_constraint(y,
                               self.xf[:self.n_dimensions],
                               self.xf[:self.n_dimensions],
                               constraint_type='final',
                               name=["yf"] * self.n_dimensions)

        # For now only punish the deviance from y_astar
        cost = 0
        for i in range(len(y)):
            cost += (y[i] - y_astar[i])**2
        self.J = definite_integral(cost, 0, 1)

        for i, obstacle in enumerate(self.obstacles):
            self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
                                                name="obst_" + str(i),
                                                constraint_type = 'obstacle')


        # Creating solver class
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g),
                'p': vertcat(*self.P)
                }

        self.solver = nlpsol('solver', 'ipopt', prob, self.options)

        # Assembling the argument dictionary
        self.arg = {'x0' : w0_coeffs,
                   'lbx': self.lbw,
                   'ubx': self.ubw,
                   'lbg': self.lbg,
                   'ubg': self.ubg}
        solution = self.solver.call(self.arg)


        flatten = lambda t: [item for sublist in t for item in sublist]
        w_opt = solution['x'].full()
        w_opt = flatten(w_opt.tolist())
        self.astar_initials["w0"] = w_opt
        """ Ez itt hót fölösleges elv..."""
        # self.w0_initial = solution
        # self.DvX = DecisionVarX(self.w_list, self.g_list, self.lbg, self.ubg)
        # self.DvX.extract(self.w0_initial)



        # basis_y = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        # basis_a = self.define_knots(degree = 1, knot_intervals = self.knot_intervals)
        # basis_b = self.define_knots(degree = 1, knot_intervals = self.knot_intervals)
        # basis_d_tau = self.define_knots(degree = 1, knot_intervals = self.knot_intervals)


        # tmp = np.array(self.DvX.y)
        # tmp = tmp.reshape(2, -1)
        # tmp = [tmp_.tolist() for tmp_ in tmp]
        # self.astar_initials["y"] = tmp

        # # Először bontsuk le obstacle-ökre
        # # a
        # tmp = np.array(self.DvX.a)
        # tmp = tmp.reshape(len(self.obstacles), -1)
        # tmp = [tmp_.reshape(2, -1).tolist() for tmp_ in tmp]
        # self.astar_initials["a"] = tmp
        # # b
        # tmp = np.array(self.DvX.b)
        # tmp = tmp.reshape(len(self.obstacles), -1)
        # tmp = [tmp_.tolist() for tmp_ in tmp]
        # self.astar_initials["b"] = tmp
        # # d_tau
        # tmp = np.array(self.DvX.b)
        # tmp = tmp.reshape(len(self.obstacles), -1)
        # tmp = [tmp_.tolist() for tmp_ in tmp]
        # self.astar_initials["d_tau"] = tmp
        # # self.astar_initials["y"] = self.DvX.y
        # # self.astar_initials["a"] = self.DvX.a
        # # self.astar_initials["b"] = self.DvX.b
        # # self.astar_initials["d_tau"] = self.DvX.d_tau

        # y_x_initialized = BSpline(basis_y, self.astar_initials["y"][0])
        # y_y_initialized = BSpline(basis_y, self.astar_initials["y"][1])


        # fx_initialized = [y_x_initialized(t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, 100)]
        # fy_initialized = [y_y_initialized(t_).reshape(1,).tolist()[0] for t_ in np.linspace(0, 1, 100)]
        # plt.plot(fx_initialized, fy_initialized, 'b.')
        # plt.savefig(self.cwd + "/figures/astar_vehicle_"+ str(self.ID)+ ".png")

        return self

    def heuristic_initialization(self):
        # Figure out if, we are bottom, mid or top vehicle
        bit_rank = 0
        for neighbour in self.neighbours:
            if self.x0[1] < neighbour.x0[1]:
                bit_rank += 1
        if bit_rank == 0:
            bit_rank = "top"
        elif bit_rank == 1:
            bit_rank = "mid"
        elif bit_rank == 2:
            bit_rank = "bottom"
        else:
            assert False, "Wait, wut?"
        
        # Figure out if obst1 or obst2 is the bottom (obst0 is always "mid", because it is a gate.)
        # and gather their respective way-points
        
        # First, get the way-point of the gate
        gate1 = self.obstacles[0].corners
        gate2 = self.obstacles[1].corners
        gate = gate1 + gate2
        gate = np.array(gate)
        mid_gate = [np.mean(gate[:, 0]), np.mean(gate[:, 1])]
        
        # top/bottom
        mid1 = [np.mean(np.array(self.obstacles[2].corners)[:, 0]), np.mean(np.array(self.obstacles[2].corners)[:, 1])]
        mid2 = [np.mean(np.array(self.obstacles[3].corners)[:, 0]), np.mean(np.array(self.obstacles[3].corners)[:, 1])]
        
        if mid1[1] < mid2[1]:
            bottom = mid1
            top = mid2
        elif mid1[1] > mid2[1]:
            top = mid1
            bottom = mid2
        else:
            assert False, "Wait, wut?"
            
        # Okay, we have the points :)
        array_top = np.array(top)
        array_bottom = np.array(bottom)
        mid_poles = [np.mean([array_top[0], array_bottom[0]]), np.mean([array_top[1], array_bottom[1]])]
        # mid_poles = [np.mean(np.array(top)), np.mean(np.array(bottom))]
        
        
        # Deciding on what comes first(gate or obstacls)
        # x0_ = np.array(self.x0[:2])
        # mid_poles_ = np.array(mid_poles)
        # mid_gate_ = np.array(mid_gate)
        # --> distance only in the x dimension
        x0_ = np.array(self.x0[0])
        mid_poles_ = np.array(mid_poles[0])
        mid_gate_ = np.array(mid_gate[0])
        
        if np.linalg.norm(x0_ - mid_poles_) < np.linalg.norm(x0_ - mid_gate_):
            # We are close to the poles
            sequence = "poles first"
        elif np.linalg.norm(x0_ - mid_poles_) > np.linalg.norm(x0_ - mid_gate_):
            sequence = "gate first"
        else:
            assert False, "Wait, wut?"
            
            
        # Let's go even lower or even higher :))
        bottom = [bottom[0], bottom[1] - self.radious * 3]
        top = [top[0], top[1] + self.radious * 3]
        
        
        # Oh, and we also need to decide, if we should first go through
        # the gate or first between the obstacles.
        
        
        # If we are bottom, then take the bottom obstacle and put a point below it.
        if bit_rank == "bottom":
            if sequence == "gate first":
                way_points = [self.x0[:2]] + [mid_gate] + [bottom] + [self.xf[:2]]
            else:
                way_points = [self.x0[:2]] + [bottom] + [mid_gate] + [self.xf[:2]]
                
        # If we are middle, then take bottom & top and put a point in the middle
        if bit_rank == "mid":
            if sequence == "gate first":
                way_points = [self.x0[:2]] + [mid_gate] + [mid_poles] + [self.xf[:2]]
            else:
                way_points = [self.x0[:2]] + [mid_poles] + [mid_gate] + [self.xf[:2]]
        
        # If we are top, then take the top obstacle and put a point above it.
        if bit_rank == "top":
            if sequence == "gate first":
                way_points = [self.x0[:2]] + [mid_gate] + [top] + [self.xf[:2]]
            else:
                way_points = [self.x0[:2]] + [top] + [mid_gate] + [self.xf[:2]]
        
        
        # Fit a spline on x0, gate center, "bottom"/"mid"/"top", xf
        spline_fitting_ON = True
        
        # --- Okay, from here on, it is pretty much the same (or at least should be...) as the astar_path initialization.
        fitter = SplineFitter(knot_intervals = self.knot_intervals)
        x = np.array(way_points)[:, 0]
        y = np.array(way_points)[:, 1]
        y_heuristic = fitter.fitting(x, y)
        self.heuristic_initials["y_heuristic"] = y_heuristic
        
        # Fill w0_coeffs with the coefficients of y_astar
        w0_coeffs = [] + y_heuristic[0].coeffs.reshape(1, -1).tolist()[0] \
            + y_heuristic[1].coeffs.reshape(1, -1).tolist()[0]
        
        # plt.figure()
        t = np.linspace(0, 1, 100)
        x_ = [y_heuristic[0](t_).reshape(-1).tolist()[0] for t_ in t]
        y_ = [y_heuristic[1](t_).reshape(-1).tolist()[0] for t_ in t]
        # plt.plot(x_, y_, 'k.')
        # plt.plot(y_heuristic[0].coeffs, y_heuristic[1].coeffs, 'ro')
        # plt.plot(x, y, 'bo')
        # plt.show()
        
        kappa = True
        
        for obstacle in self.obstacles:
            x_c, y_c = [np.mean(np.array(obstacle.corners)[:, 0]), np.mean(np.array(obstacle.corners)[:, 1])]
            a_x, a_y = [], []
            for i in range(len(x)):
                a_x += [x_c - x[i]]
                a_y += [y_c - y[i]]
            a_spline = fitter.fitting(a_x, a_y, degree = 1)
            xy_component = [spline.coeffs.reshape(1, -1).tolist()[0] for spline in a_spline]
            a_coeffs = xy_component[0] + xy_component[1]
            # a_coeffs = [0] * len(a_coeffs)
            b_coeffs = [0] * len(a_spline[0].basis)
            d_tau_coeffs = [self.epsilon] * len(a_spline[0].basis)
            # assembling w0_coeffs
            w0_coeffs += a_coeffs + b_coeffs + d_tau_coeffs
            
        # ------
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []
        # Okay. We have the path. Now we want find the corresponding a, b, d_tau values
        # But how do we do that?
        # Step 1: get y_astar
        # (we have done so above)

        # plt.savefig(self.cwd + "/figures/astar_vehicle_"+ str(self.ID)+ ".png")
        # Step 2: y shluld be close to y_astar

        # Step 3: apply constarints on y
        # Step 4: optimize
        # Step 5: recode def define_MX_spline() such that it accepts entire
        # initial values, not just their starting point and ending point.

        y = self.define_MX_spline(degree=self.state_degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                initial_value = [y_heuristic[0].coeffs.reshape(-1,).tolist(), y_heuristic[0].coeffs.reshape(-1,).tolist()],
                                name=["y"] * self.n_dimensions)

        # Initial position constraint on y
        self.define_constraint(y,
                               self.x0[:self.n_dimensions],
                               self.x0[:self.n_dimensions],
                               constraint_type='initial',
                               name=["y0"] * self.n_dimensions)


        # Final position constraint on y
        self.define_constraint(y,
                               self.xf[:self.n_dimensions],
                               self.xf[:self.n_dimensions],
                               constraint_type='final',
                               name=["yf"] * self.n_dimensions)

        # For now only punish the deviance from y_astar
        cost = 0
        for i in range(len(y)):
            cost += (y[i] - y_heuristic[i])**2
        self.J = definite_integral(cost, 0, 1)

        for i, obstacle in enumerate(self.obstacles):
            self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
                                                name="obst_" + str(i),
                                                constraint_type = 'obstacle')


        # Creating solver class
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g),
                'p': vertcat(*self.P)
                }

        self.solver = nlpsol('solver', 'ipopt', prob, self.options)

        # Assembling the argument dictionary
        self.arg = {'x0' : w0_coeffs,
                   'lbx': self.lbw,
                   'ubx': self.ubw,
                   'lbg': self.lbg,
                   'ubg': self.ubg}
        solution = self.solver.call(self.arg)


        flatten = lambda t: [item for sublist in t for item in sublist]
        w_opt = solution['x'].full()
        w_opt = flatten(w_opt.tolist())
        self.heuristic_initials["w0"] = w_opt
        
        return self

    def set_position(self, position : list, position_type : str):
        # 2D
        if position_type == 'initial':
            self.x0 = position + [0, 0]
        elif position_type == 'final':
            self.xf = position + [0, 0]
        else:
            raise NotImplementedError()

        return self


    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################

    def update_PvZ(self):
        """Updating P0_z parameter. Values, that are commented out are not
        currently updated. This can be changed later allowing additional functionality.
        Updated values are: x_i, x_j
        """
        self.PvZ.y = self.DvX.y
        try:
            self.PvZ.y_j = self.message_in['y_j']
        except:
            pass

        return self

    def update_PvX(self):
        """Updating P0 parameter. Values, that are commented out are not
        currently updated. This can be changed later allowing additional functionality.
        Updated values are: T, x0, z_i, z_ji, lambda_ji
        """
        self.PvX.z_i = self.DvZ.z_i
        try:
            self.PvX.z_ji = self.message_in['z_ji']
            self.PvX.lambda_ji = self.message_in['lambda_ji']
        except:
            pass


        return self
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    "Data exchange"


    def data_exchange_z_send(self):
        """Sharing lambda_ij, z_ij data

        Parameters
        ----------
        none

        Returns
        message : list
            form: [{'sender' : senderID, 'receiver': receiverID, 'data_name': data}]
        -------
        """
        lambda_ij = np.array(self.PvZ.lambda_ij).reshape((len(self.neighbours), -1))
        z_ij = np.array(self.DvZ.z_ij).reshape((len(self.neighbours), -1))

        message = []
        for i in range(len(self.neighbours)):
            message += [{'sender': self.ID, 'receiver': self.neighbours[i].ID, 'lambda_ij': lambda_ij[i, :].tolist(), \
                                      'z_ij': z_ij[i, :].tolist()} ]

        return message

    def data_exchange_z_receive(self, message):
        """Receiving message to update: lambda_ji and z_ji values
        incoming: lambda_ij -> updated: lambda_ji
        incoming: z_ij -> updated: z_ji

        Parameters
        ----------
        message : list
            form: [{'sender' : senderID, 'receiver': receiverID, 'data_name': data}]

        Returns
        self
        -------
        """

        lambda_ji = []
        z_ji = []

        for neighbour_ in self.neighbours:
            for dict_ in message:
                if neighbour_.ID == dict_['sender'] and self.ID == dict_['receiver']:
                    lambda_ji += dict_['lambda_ij']
                    z_ji += dict_['z_ij']

        self.message_in['lambda_ji'] = lambda_ji
        self.message_in['z_ji'] = z_ji

        return self


    def data_exchange_x_send(self):
        """Sharing our own x_i data
        receiverID = -1, because we send it to everyone. In the receive function,
        every agent ignores messages that are not sent from neighboring agents.

        Parameters
        ----------
        none

        Returns
        message : list
            form: [{'sender' : senderID, 'receiver': receiverID = -1, 'data_name': data}]
        -------
        """
        message = [{'sender': self.ID, 'receiver': -1, 'y_i': self.DvX.y}] # the first 4 belongs to x0. We don't optimize for that in the z update

        return message


    def data_exchange_x_receive(self, message):
        """Receiving message to update: x_j values
        receiverID = -1, because x_j is sent to everyone. We ignore messages
        that are not sent from neighboring agents.

        Parameters
        ----------
        message : list
            form: [{'sender' : senderID, 'receiver': receiverID = -1, 'data_name': data}]

        Returns
        self
        -------
        """
        y_j = []
        for neighbour_ in self.neighbours:
            for dict_ in message:
                if neighbour_.ID == dict_['sender']:
                    y_j += dict_['y_i']

        # y_j = np.array(y_j).reshape(len(self.neighbours), -1)
        # self.message_in['y_j'] = self.unfold(y_j, self.neighbours)
        self.message_in['y_j'] = y_j

        # Saving some variables
        self.variable_history['y_j'] += [y_j]
        return self

    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    "The folding functions"

    # def unfold(self, matrix, neighbours : list):
    #     """A helper function, used in other functions, where the format of the
    #     data has to be changed. Its "inverse" function is fold().
    #     unfold() takes a matrix of shape(n_neighbours, 4 * N) and transforms it into
    #     shape(1, n_neighbours * 4 * N)
    #     The resulting vector contains data, where the 1st 4 values are from neighbour 1,
    #     the 2nd 4 are from neighbour 2 etc. This is the format in which data is
    #     stored in the P0 and P0_z parameters and this is how the optimizer at
    #     its current shape expects information.
    #     """

    #     # Takes a matrix of np.array(n_neighbours, N * 4) and unfolds it such that it
    #     # takes consecutively 4 values from each row
    #     list_ = []
    #     # flatten = lambda t: [item for sublist in t for item in sublist]
    #     for i in range(int(matrix.shape[1] / self.state_len)):
    #         idx = np.arange(self.state_len*i,self.state_len*i+self.state_len) # 4 values, step by step
    #         for j in range(len(neighbours)):
    #             list_ += matrix[j, idx].tolist()

    #     return list_

    # def fold(self, vector : list, neighbours : list):
    #     """A helper function, used in other functions, where the format of the
    #     data has to be changed. Its "inverse" function is unfold().
    #     fold() takes a vector of shape(1, n_neighbours * 4 * N) and folds it into
    #     a matrix of shape(n_neighbours, 4 * N).

    #     The resulting rows of the matrix contain data associated with the given
    #     neighbour agent. This is the format in which data is stored in the messages or
    #     this format is used when some computation is taking place, like updating
    #     the lambda values.
    #     """

    #     # Takes a vector
    #     # Unfolds into np.array(n_neighbours, N * 4)
    #     matrix = np.array(vector).reshape(len(neighbours), -1) * 0
    #     tmp_matrix = np.array(vector).reshape(-1, self.state_len)

    #     for i in range(len(neighbours)):
    #         matrix[i, :] = tmp_matrix[i::len(neighbours), :].reshape(1, -1)

    #     return matrix

    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    def lambda_update(self):
        """Updating lambda_i and lambda_ij values (in P0 and P0_z)"""
        lambda_i_old = np.array(self.PvZ.lambda_i)
        # lambda_i_new = lambda_i_old * 0
        y = np.array(self.DvX.y)
        z_i = np.array(self.DvZ.z_i)

        lambda_ij_old = np.array(self.PvZ.lambda_ij)
        # lambda_ij_new = lambda_ij_old * 0
        y_j = np.array(self.message_in['y_j'])
        z_ij = np.array(self.DvZ.z_ij)

        lambda_i_new = lambda_i_old + self.rho * (y - z_i)
        lambda_ij_new = lambda_ij_old + self.rho * (y_j - z_ij)

        self.PvX.lambda_i = lambda_i_new.tolist()
        self.PvZ.lambda_i = lambda_i_new.tolist()
        self.PvZ.lambda_ij = lambda_ij_new.tolist()




    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################

    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    """ Okay :)
    I will put some frenet-frame code here and will decide later, what to do whit it :)
    """
    
    # def setup_z_update_frenet(self):
        
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################

    def prepare0(self):
        "TODO"

    def prepare1(self):

        self.setup_x_update()
        self.setup_z_update()

    def prepare2(self):
        "TODO"

    def setup_z_update(self):
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []
        


        # Define ADMM cost
        # x_i - z_i
        y = self.define_MX_spline(degree=self.state_degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                name=["y"] * self.n_dimensions,
                                category = 'parameter')

        z_i = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound=self.y_min, upper_bound=self.y_max,
                               initial_value = [[self.x0[0], self.xf[0]], [self.x0[1], self.xf[1]]],
                               name = ['z_i'] * self.n_dimensions)

        lambda_i  = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound = [], upper_bound = [],
                               name = ['lambda_i'] * self.n_dimensions,
                               category = 'parameter')
        
        # Frenet
        p_sum = 0
        q_sum = 0
        if self.frenet_enabled:
            p_sum += z_i[0]
            q_sum += z_i[1]
            

        for i in range(len(y)):
            self.J += definite_integral(lambda_i[i] * (y[i] - z_i[i]), 0, 1)
            self.J += definite_integral(self.rho * (y[i] - z_i[i])**2, 0, 1)

        # Cost sum: x_j - z_ij
        for i in range(len(self.neighbours)):

            y_j = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound = [], upper_bound = [],
                               name = ['y_j'] * self.n_dimensions,
                               category = 'parameter')

            z_ij = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                                   lower_bound=self.y_min, upper_bound=self.y_max,
                                   initial_value = [[self.neighbours[i].x0[0], self.neighbours[i].xf[0]], [self.neighbours[i].x0[1], self.neighbours[i].xf[1]]],
                                   name = ['z_ij'] * self.n_dimensions)
            lambda_ij  = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                                   lower_bound = [], upper_bound = [],
                                   name = ['lambda_ij'] * self.n_dimensions,
                                   category = 'parameter')


            for j in range(len(y)):
                self.J += definite_integral(lambda_ij[j] * (y_j[j] - z_ij[j]), 0, 1)
                self.J += definite_integral(self.rho * (y_j[j] - z_ij[j])**2, 0, 1)

            # Define hard formation constraints
            # self.define_constraint(y - z_ij,
            #                        np.array(self.xf[:self.n_dimensions]) - np.array(self.neighbours[i].xf[:self.n_dimensions]),
            #                        np.array(self.xf[:self.n_dimensions]) - np.array(self.neighbours[i].xf[:self.n_dimensions]),
            #                        constraint_type='overall',
            #                        name=["formation_vehicle_" + str(i)] * self.n_dimensions)

            def cross_product(spline1, spline2):
                a1, a2 = spline1[0], spline1[1]
                b1, b2 = spline2[0], spline2[1]
                return (a1 * b2 - a2 * b1)**2

            def dot_product(spline1, spline2):
                a1, a2 = spline1[0], spline1[1]
                b1, b2 = spline2[0], spline2[1]
                return (a1 * b1 + a2 * b2)

            def len_square(spline1):
                a1, a2 = spline1[0], spline1[1]
                return a1**2 + a2**2
            
            # def rotate_spline(spline2D):
            #     spline2D[0]

            # vec1: what is should be
            # vec2: what we have
            # cross: the cross product of the two vectors. It is a function of t.
            # dot: not used
            # vec1_length, vec2_length: not used
            vec1 = z_i - z_ij
            vec2 = np.array(self.xf[:self.n_dimensions]) - np.array(self.neighbours[i].xf[:self.n_dimensions])
            cross = cross_product(vec1, vec2)
            dot = dot_product(vec1, vec2)
            vec1_length = len_square(vec1)
            vec2_length = len_square(vec2)

            # fitter = SplineFitter(knot_intervals = self.knot_intervals)
            # rotated_vec2x = []
            # rotated_vec2y = []
            # theta = np.linspace(0, 2 * math.pi, self.t_resolution_length)
            # for j in range(self.t_resolution_length):
            #     x_new, y_new = self.plot_rotation(vec2[0], vec2[1], theta[j])
            #     rotated_vec2x += [x_new]
            #     rotated_vec2y += [y_new]

            # rotated_vec2 = fitter.fitting(rotated_vec2x, rotated_vec2y)
            # cross = cross_product(vec1, rotated_vec2)

            # x, y = [], []
            # for t in np.linspace(0, 1, self.t_resolution_length):
            #     x += [rotated_vec2[0](t)[0][0]]
            #     y += [rotated_vec2[1](t)[0][0]]

            # plt.plot(x, y)
            # plt.savefig("tmp.png")

            # "normalization" attempt
            # vec1 = vec1 / definite_integral(vec1[0]**2 + vec1[1]**2, 0, 1)
            # vec2 = vec2 / definite_integral(vec2[0]**2 + vec2[1]**2, 0, 1)

            # cross = cross / definite_integral(cross, 0, 1)

            # Formation constraint.
            for t in np.linspace(0, 1, self.t_resolution_length):
                self.define_constraint([cross(t)],
                                        [-self.slack * 1],
                                        [self.slack * 1],
                                        constraint_type='time',
                                        name=["formation_vehicle_" + str(i)] * self.n_dimensions)
            # But the dot product should be > 0, to avoid the vehicles switching place and still
            # fulfilling the formation requirements (at least for those two vehicles)

            # for t in np.linspace(0, 1, self.t_resolution_length):
            #     self.define_constraint([dot(t)],
            #                             [0],
            #                             [math.inf],
            #                             constraint_type='time',
            #                             name=["formation_dot_vehicle_" + str(i)] * self.n_dimensions)

            # TODO: I think this is not really needed anymore, but need to check
            # for j in range(len(y)):
            #     self.J += self.rho_formation * definite_integral( ((vec1[j] - vec2[j])**2), 0, 1)


            """Special distance-constraint"""
            """Special distance-constraint"""
            frenet_zero = MX((0, 0))
            xf = np.array(self.xf[:self.n_dimensions])
            xf_j = np.array(self.neighbours[i].xf[:self.n_dimensions])

            # dist_we_have = (z_i[0] - frenet_zero[0])**2 \
            #                 + (z_i[1] - frenet_zero[1])**2
            # dist_we_want = (xf[0] - frenet_zero[0])**2 \
            #                 + (xf[1] - frenet_zero[1])**2
            dist_we_have = (z_i[0] - z_ij[0])**2 \
                            + (z_i[1] - z_ij[1])**2
            dist_we_want = (xf[0] - xf_j[0])**2 \
                            + (xf[1] - xf_j[1])**2
            dist_difference = (dist_we_have * 1 - dist_we_want * 0.25) * 1  # 0.5 means we can shrink to the quarter of the size

            ""
            for t in np.linspace(0, 1, self.t_resolution_length):
                self.define_constraint([dist_difference(t)],
                                        [0.0],
                                        [math.inf],
                                        constraint_type='time',
                                        name=["formation_vehicle_" + str(i)] * self.n_dimensions)

                # We can add collision avoidance here too :)
                self.define_constraint([dist_we_have(t)],
                                        [(self.radious * 2 * self.vehicle_avoidnce_multiplier)**2],
                                        [math.inf],
                                        constraint_type='time',
                                        name=["formation_vehicle_" + str(i)] * self.n_dimensions)
            """Special distance-constraint"""
            """Special distance-constraint"""
                # or:
                # self.J += self.rho_formation * definite_integral( (dist_we_have - self.radious*2)**2, 0, 1)

                # self.J += self.rho_formation * definite_integral( (((vec1 - vec2)**2)[j] - self.radious**2)**2, 0, 1)


            # Inter-vehicle collision avoidance only at prescribed time-instances
            # for i, neighbour in enumerate(self.neighbours):
            #     self.collision_avoidance_hyperplane(z_i, z_ij, radious = self.radious*2.5,
            #                                         name = "inter_vehicle" + str(i),
            #                                         constraint_type="inter_vehicle_t",
            #                                         n_samples = 20)

            # We add an extra constraint, if we are in the Frenet-frame.
            if self.frenet_enabled:
                # Péni féle
                # If we add the p components and if we add the q components together, 
                # each of these should equal to zero. Meaning, the center of gravity 
                # should be in the (0, 0) point of the Frenet-frame.
                
                # This is defined outside of this loop :)
                # p_sum = z_i[0]
                # q_sum = z_i[1]
                
                p_sum += z_ij[0]
                q_sum += z_ij[1]
                
                
                    
        if self.frenet_enabled:
            for t in np.linspace(0, 1, self.t_resolution_length):
                self.define_constraint([p_sum(t), q_sum(t)],
                                        [-self.slack,-self.slack],
                                        [ self.slack, self.slack],
                                        constraint_type='time',
                                        name=["frenet_zero_" + str(i)] * self.n_dimensions)


        # Containers
        self.DvZ = DecisionVarZ(self.w_list, self.g_list, self.lbg, self.ubg)
        self.PvZ = ParamValZ(self.P_list, self.P0)

        # Initializing values
        self.DvZ.extract(self.w0)


        "Initialize values with the pre-calculated values from initialize_x()"
        if self.initial_values != {}:
            self.DvZ.w0_z = self.initial_values["z_i"] + self.initial_values["z_ji"]
            self.DvZ.z_i = self.initial_values["z_i"]
            self.DvZ.z_ij = self.initial_values["z_ji"]
            self.PvZ.y = self.initial_values["y"]
            self.PvZ.y_j = self.initial_values["y_j"]
            self.PvZ.lambda_i = self.initial_values["lambda_i"]
            self.PvZ.lambda_ij = self.initial_values["lambda_ij"]
            self.message_in["y_j"] = self.initial_values["y_j"]
            self.message_in["z_ji"] = self.initial_values["z_ji"]
            self.message_in["lambda_ji"] = self.initial_values["lambda_ji"]

        # Creating solver class
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g),
                'p': vertcat(*self.P)
                }

        self.solver_z = nlpsol('solver', 'ipopt', prob, self.options_z)

        # Assembling the argument dictionary
        self.arg_z = {'x0' : self.w0,
                   'lbx': self.lbw,
                   'ubx': self.ubw,
                   'lbg': self.lbg,
                   'ubg': self.ubg,
                   'p': self.PvZ.assemble()}

        return self

    def z_update(self):
        self.update_PvZ()

        # Updating the necessary arguments for the solver
        self.arg_z['x0'] = self.DvZ.w0_z
        self.arg_z['p'] = self.PvZ.assemble()

        # Solving the problem
        t1 = time.time()
        self.solution_z = self.solver_z.call(self.arg_z)
        t2 = time.time()
        self.z_update_time += [t2-t1]

        # Extracting the solution
        self.DvZ.extract(self.solution_z)

        return self

    def initialize_x(self):
        # self.value_function_generation()
        # assert 0
        # self.gurobi_spline_path()
        # self.gurobi_path()
        # self.astar_path()
        self.heuristic_initialization()
        """ This function performs a simple trajectory optimization step,
        whereas neighbouring vehicles are not considered, hence formation is also
        not considered. It is used to initialize all ADMM decision variables and
        parameters later on.
        """
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []


        y = self.define_MX_spline(degree=self.state_degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                initial_value = [[self.x0[0], self.xf[0]], [self.x0[1], self.xf[1]]],
                                name=["y"] * self.n_dimensions)

        # Initial position constraint on y
        self.define_constraint(y,
                               self.x0[:self.n_dimensions],
                               self.x0[:self.n_dimensions],
                               constraint_type='initial',
                               name=["y0"] * self.n_dimensions)


        # Final position constraint on y
        self.define_constraint(y,
                               self.xf[:self.n_dimensions],
                               self.xf[:self.n_dimensions],
                               constraint_type='final',
                               name=["yf"] * self.n_dimensions)

        # Centralised
        dy = [y_.derivative() for y_ in y]
        u = [dy_.derivative() for dy_ in dy]

        # Cost function
        cost = 0
        for i in range(len(u)):
            cost += u[i]**2
        self.J += self.rho_input * definite_integral(cost, 0, 1)

        # Collision avoidance

        for i, obstacle in enumerate(self.obstacles):
            self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
                                                name="obst_" + str(i),
                                                constraint_type = 'obstacle')


        # Creating solver class
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g),
                'p': vertcat(*self.P)
                }

        self.solver = nlpsol('solver', 'ipopt', prob, self.options)

        if self.astar_initials != {}:
            self.w0 = self.astar_initials["w0"]
        if self.heuristic_initials != {}:
            self.w0 = self.heuristic_initials["w0"]
        self.arg = {'x0' : self.w0,
                   'lbx': self.lbw,
                   'ubx': self.ubw,
                   'lbg': self.lbg,
                   'ubg': self.ubg}
        self.w0_initial = self.solver.call(self.arg)


        # This is needed, because data_exchange_x uses self.DvX.y
        # The variablen will be deleted at the end of the initialization process
        # for good measure.
        self.DvX = DecisionVarX(self.w_list, self.g_list, self.lbg, self.ubg)
        self.DvX.extract(self.w0_initial)

        # --> y, a, b, d_tau is done
        # - data_exchange
        self.initial_values["w0_initial"] = self.w0_initial
        self.initial_values["DvX"] = self.DvX


        return self

    def initialize_values(self):
        """This function initializes saves some values into a dictionary.
        These values will be used to initialize all decision variables & parameters
        for the x & z update respectively. (in setup_x_update() and setup_z_update() )"""

        self.initial_values["y"] = self.DvX.y
        self.initial_values["z_i"] = self.DvX.y
        self.initial_values["z_ji"] = self.DvX.y * len(self.neighbours)
        self.initial_values["y_j"] = self.message_in["y_j"]
        self.initial_values["z_j"] = self.message_in["y_j"]
        self.initial_values["z_ij"] = self.message_in["y_j"]
        self.initial_values["lambda_i"] = [1] * len(self.DvX.y)
        self.initial_values["lambda_ij"] = [1] * len(self.DvX.y) * len(self.neighbours)
        self.initial_values["lambda_ji"] = [1] * len(self.DvX.y) * len(self.neighbours)

        # Setting back every variable that belong to the ADMM iterations
        # (this step is actually not necessary)
        self.DvX = []
        self.message_in = {}
        self.variable_history =  {'y' : [],         # x_update
                                  'y_j' : [],       # data_exchange_x_receive
        }

        return self


    def setup_x_update_FRENET(self):
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []

        y = self.define_MX_spline(degree=self.state_degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                initial_value = [[self.x0[0], self.xf[0]], [self.x0[1], self.xf[1]]],
                                name=["y"] * self.n_dimensions)
        # Centralised
        dy = [y_.derivative() for y_ in y]
        u = [dy_.derivative() for dy_ in dy]

        # TODO: overall constraints on y, dy and u
        # - bounds on y are set upon creation
        # - no bounds are set for dy
        # - bounds on u are set below

        # Input constraint
        self.define_constraint(u,
                               self.u_min,
                               self.u_max,
                               constraint_type='overall',
                               name=["input_constraint"] * self.n_dimensions)



        # Initial position constraint on y
        self.define_constraint(y,
                               self.x0[:self.n_dimensions],
                               self.x0[:self.n_dimensions],
                               constraint_type='initial',
                               name=["y0"] * self.n_dimensions)
        # Initial velocity constraint on dy
        self.define_constraint(dy,
                               self.x0[self.n_dimensions:self.n_dimensions*2],
                               self.x0[self.n_dimensions:self.n_dimensions*2],
                               constraint_type='initial',
                               name=["dy0"] * self.n_dimensions)

        # Final position constraint on y
        self.define_constraint(y,
                               self.xf[:self.n_dimensions],
                               self.xf[:self.n_dimensions],
                               constraint_type='final',
                               name=["yf"] * self.n_dimensions)

        # Final velocity constraint on dy
        self.define_constraint(dy,
                               self.xf[self.n_dimensions:self.n_dimensions*2],
                               self.xf[self.n_dimensions:self.n_dimensions*2],
                               constraint_type='final',
                               name=["dyf"] * self.n_dimensions)



        from frenet_path import FrenetPath
        self.fp = FrenetPath(x0 = -5, xf = 5, N = self.N)
        v_s = self.fp.fx_
        
        # Collision avoidance with obstacles
        # for i, obstacle in enumerate(self.obstacles):
        #     self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
        #                                         name="obst_" + str(i),
        #                                         constraint_type = 'obstacle')
                                                # n_samples = self.t_resolution_length)

        # Collision avoidance with obstacles, where the center point is the mean
        # value of the corner poistions and the radious is assumed to be obstacle.size
        # for i, obstacle in enumerate(self.obstacles):
        #     center = []
        #     center += [np.mean(np.array(obstacle.corners)[:, 0])]
        #     center += [np.mean(np.array(obstacle.corners)[:, 1])]

        #     np_center = np.array(center)
        #     np_corner = np.array(obstacle.corners[0])
        #     max_extent = np.sqrt(  (center[0] - np_corner[0])**2 + (center[1] - np_corner[1])**2 )
            # max_extent = float(max_extent)
            # self.collision_avoidance_circular(y, center, radious=self.radious + max_extent,
            #                                     name="obst_" + str(i))
            # self.collision_avoidance_circular(y, center, radious=max_extent-0.00,
            #                                     name="obst_" + str(i))

        # Collision avoidance with stationary obstacles only at prescribed time-instances
        # for i, obstacle in enumerate(self.obstacles):
        #     self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
        #                                         name="obst_" + str(i),
        #                                         constraint_type = 'spline_obstacle_t',
        #                                         n_samples = 10)
        for i, obstacle in enumerate(self.obstacles):
            self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious*self.obstacle_avoidance_multiplier,
                                                name="obst_" + str(i),
                                                constraint_type = 'obstacle')






        # Cost function
        cost = 0
        for i in range(len(u)):
            cost += u[i]**2
        self.J += self.rho_input * definite_integral(cost, 0, 1)

        # Encourage them to move towards the final position
        # cost = 0
        # for t in np.linspace(0, 1, self.t_resolution_length):
        #     cost += (y[0](t) - np.array(self.xf[0]) )**2 + (y[1](t) - np.array(self.xf[1]) )**2
        # self.J += cost


        # Cost x_i - z_i
        z_i = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound = [], upper_bound = [],
                               name = ['z_i'] * self.n_dimensions,
                               category = 'parameter')
        lambda_i  = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound = [], upper_bound = [],
                               name = ['lambda_i'] * self.n_dimensions,
                               category = 'parameter')


        for i in range(len(y)):
            self.J += definite_integral(lambda_i[i] * (y[i] - z_i[i]), 0, 1)
            self.J += definite_integral(self.rho * (y[i] - z_i[i])**2, 0, 1)

        # Cost sum: x_i - z_ji
        for i in range(len(self.neighbours)):
            z_ji = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                                   lower_bound = [], upper_bound = [],
                                   name = ['z_ji'] * self.n_dimensions,
                                   category = 'parameter')
            lambda_ji  = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                                   lower_bound = [], upper_bound = [],
                                   name = ['lambda_ji'] * self.n_dimensions,
                                   category = 'parameter')

            for j in range(len(y)):
                self.J += definite_integral(lambda_ji[j] * (y[j] - z_ji[j]), 0, 1)
                self.J += definite_integral(self.rho * (y[j] - z_ji[j])**2, 0, 1)





            # # Inter-vehicle collision avoidance
            # for i, neighbour in enumerate(self.neighbours):
            #     self.collision_avoidance_hyperplane(y, z_ji, radious = self.radious / 1e5,
            #                                         name = "inter_vehicle" + str(i),
            #                                         constraint_type="inter_vehicle_t")


        # Containers
        self.DvX = DecisionVarX(self.w_list, self.g_list, self.lbg, self.ubg)
        self.PvX = ParamValX(self.P_list, self.P0)

        # Initializing values - below this is overwritten by the initial values
        # found during initialize_x()
        self.DvX.extract(self.w0)

        "Initialize values with the pre-calculated values from initialize_x()"
        if self.initial_values != {}:
            self.DvX.extract(self.initial_values["w0_initial"])
            self.PvX.z_i = self.initial_values["z_i"]
            self.PvX.z_ji = self.initial_values["z_ji"]
            self.PvX.lambda_i = self.initial_values["lambda_i"]
            self.PvX.lambda_ji = self.initial_values["lambda_ji"]
            self.message_in["y_j"] = self.initial_values["y_j"]
            self.message_in["z_ji"] = self.initial_values["z_ji"]
            self.message_in["lambda_ji"] = self.initial_values["lambda_ji"]


        # Creating solver class
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g),
                'p': vertcat(*self.P)
                }

        self.solver = nlpsol('solver', 'ipopt', prob, self.options)

        # # Initialize some values IF self.initial_values is not empty
        # if self.initial_values != {}:
        #     # DvX already contains the initial y, a, b, d_tau values
        #     # we need to initialize PvX: lambda_i, z_i, lambda_ji, z_ji
        #     self.PvX.lambda_i = self.initial_values["lambda_i"]
        #     self.PvX.z_i = self.initial_values["z_i"]
        #     self.PvX.lambda_ji = self.initial_values["lambda_ji"]
        #     self.PvX.z_ji = self.initial_values["z_ji"]

        # Assembling the argument dictionary
        self.arg = {'x0' : self.w0,
        # self.arg = {'x0' : [0] * len(self.w0),
        # self.arg = {'x0' : self.w0_initial['x'].full().reshape(-1,).tolist(),
                   'lbx': self.lbw,
                   'ubx': self.ubw,
                   'lbg': self.lbg,
                   'ubg': self.ubg,
                   'p': self.PvX.assemble()}

        return self
    def setup_x_update(self):
        self.w, self.lbw, self.ubw = [], [], []
        self.g, self.lbg, self.ubg = [], [], []
        self.J = 0
        self.P = []
        self.P0 = []
        self.w0 = []
        self.w_list, self.g_list, self.P_list = [], [], []

        y = self.define_MX_spline(degree=self.state_degree, knot_intervals=self.knot_intervals, n_spl=self.n_dimensions,
                                lower_bound=self.y_min, upper_bound=self.y_max,
                                initial_value = [[self.x0[0], self.xf[0]], [self.x0[1], self.xf[1]]],
                                name=["y"] * self.n_dimensions)
        # Centralised
        dy = [y_.derivative() for y_ in y]
        u = [dy_.derivative() for dy_ in dy]

        # TODO: overall constraints on y, dy and u
        # - bounds on y are set upon creation
        # - no bounds are set for dy
        # - bounds on u are set below

        # Input constraint
        self.define_constraint(u,
                               self.u_min,
                               self.u_max,
                               constraint_type='overall',
                               name=["input_constraint"] * self.n_dimensions)



        # Initial position constraint on y
        self.define_constraint(y,
                               self.x0[:self.n_dimensions],
                               self.x0[:self.n_dimensions],
                               constraint_type='initial',
                               name=["y0"] * self.n_dimensions)
        # Initial velocity constraint on dy
        self.define_constraint(dy,
                               self.x0[self.n_dimensions:self.n_dimensions*2],
                               self.x0[self.n_dimensions:self.n_dimensions*2],
                               constraint_type='initial',
                               name=["dy0"] * self.n_dimensions)

        # Final position constraint on y
        self.define_constraint(y,
                               self.xf[:self.n_dimensions],
                               self.xf[:self.n_dimensions],
                               constraint_type='final',
                               name=["yf"] * self.n_dimensions)

        # Final velocity constraint on dy
        self.define_constraint(dy,
                               self.xf[self.n_dimensions:self.n_dimensions*2],
                               self.xf[self.n_dimensions:self.n_dimensions*2],
                               constraint_type='final',
                               name=["dyf"] * self.n_dimensions)



        # Collision avoidance with obstacles
        # for i, obstacle in enumerate(self.obstacles):
        #     self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
        #                                         name="obst_" + str(i),
        #                                         constraint_type = 'obstacle')
                                                # n_samples = self.t_resolution_length)

        # Collision avoidance with obstacles, where the center point is the mean
        # value of the corner poistions and the radious is assumed to be obstacle.size
        # for i, obstacle in enumerate(self.obstacles):
        #     center = []
        #     center += [np.mean(np.array(obstacle.corners)[:, 0])]
        #     center += [np.mean(np.array(obstacle.corners)[:, 1])]

        #     np_center = np.array(center)
        #     np_corner = np.array(obstacle.corners[0])
        #     max_extent = np.sqrt(  (center[0] - np_corner[0])**2 + (center[1] - np_corner[1])**2 )
            # max_extent = float(max_extent)
            # self.collision_avoidance_circular(y, center, radious=self.radious + max_extent,
            #                                     name="obst_" + str(i))
            # self.collision_avoidance_circular(y, center, radious=max_extent-0.00,
            #                                     name="obst_" + str(i))

        # Collision avoidance with stationary obstacles only at prescribed time-instances
        # for i, obstacle in enumerate(self.obstacles):
        #     self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious,
        #                                         name="obst_" + str(i),
        #                                         constraint_type = 'spline_obstacle_t',
        #                                         n_samples = 10)
        for i, obstacle in enumerate(self.obstacles):
            self.collision_avoidance_hyperplane(y, obstacle.corners, radious=self.radious*self.obstacle_avoidance_multiplier,
                                                name="obst_" + str(i),
                                                constraint_type = 'obstacle')






        # Cost function
        cost = 0
        for i in range(len(u)):
            cost += u[i]**2
        self.J += self.rho_input * definite_integral(cost, 0, 1)

        # Encourage them to move towards the final position
        # cost = 0
        # for t in np.linspace(0, 1, self.t_resolution_length):
        #     cost += (y[0](t) - np.array(self.xf[0]) )**2 + (y[1](t) - np.array(self.xf[1]) )**2
        # self.J += cost


        # Cost x_i - z_i
        z_i = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound = [], upper_bound = [],
                               name = ['z_i'] * self.n_dimensions,
                               category = 'parameter')
        lambda_i  = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                               lower_bound = [], upper_bound = [],
                               name = ['lambda_i'] * self.n_dimensions,
                               category = 'parameter')


        for i in range(len(y)):
            self.J += definite_integral(lambda_i[i] * (y[i] - z_i[i]), 0, 1)
            self.J += definite_integral(self.rho * (y[i] - z_i[i])**2, 0, 1)

        # Cost sum: x_i - z_ji
        for i in range(len(self.neighbours)):
            z_ji = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                                   lower_bound = [], upper_bound = [],
                                   name = ['z_ji'] * self.n_dimensions,
                                   category = 'parameter')
            lambda_ji  = self.define_MX_spline(degree = self.state_degree, knot_intervals = self.knot_intervals, n_spl = self.n_dimensions,
                                   lower_bound = [], upper_bound = [],
                                   name = ['lambda_ji'] * self.n_dimensions,
                                   category = 'parameter')

            for j in range(len(y)):
                self.J += definite_integral(lambda_ji[j] * (y[j] - z_ji[j]), 0, 1)
                self.J += definite_integral(self.rho * (y[j] - z_ji[j])**2, 0, 1)





            # # Inter-vehicle collision avoidance
            # for i, neighbour in enumerate(self.neighbours):
            #     self.collision_avoidance_hyperplane(y, z_ji, radious = self.radious / 1e5,
            #                                         name = "inter_vehicle" + str(i),
            #                                         constraint_type="inter_vehicle_t")


        # Containers
        self.DvX = DecisionVarX(self.w_list, self.g_list, self.lbg, self.ubg)
        self.PvX = ParamValX(self.P_list, self.P0)

        # Initializing values - below this is overwritten by the initial values
        # found during initialize_x()
        self.DvX.extract(self.w0)

        "Initialize values with the pre-calculated values from initialize_x()"
        if self.initial_values != {}:
            self.DvX.extract(self.initial_values["w0_initial"])
            self.PvX.z_i = self.initial_values["z_i"]
            self.PvX.z_ji = self.initial_values["z_ji"]
            self.PvX.lambda_i = self.initial_values["lambda_i"]
            self.PvX.lambda_ji = self.initial_values["lambda_ji"]
            self.message_in["y_j"] = self.initial_values["y_j"]
            self.message_in["z_ji"] = self.initial_values["z_ji"]
            self.message_in["lambda_ji"] = self.initial_values["lambda_ji"]


        # Creating solver class
        prob = {'f': self.J,
                'x': vertcat(*self.w),
                'g': vertcat(*self.g),
                'p': vertcat(*self.P)
                }

        self.solver = nlpsol('solver', 'ipopt', prob, self.options)

        # # Initialize some values IF self.initial_values is not empty
        # if self.initial_values != {}:
        #     # DvX already contains the initial y, a, b, d_tau values
        #     # we need to initialize PvX: lambda_i, z_i, lambda_ji, z_ji
        #     self.PvX.lambda_i = self.initial_values["lambda_i"]
        #     self.PvX.z_i = self.initial_values["z_i"]
        #     self.PvX.lambda_ji = self.initial_values["lambda_ji"]
        #     self.PvX.z_ji = self.initial_values["z_ji"]

        # Assembling the argument dictionary
        self.arg = {'x0' : self.w0,
        # self.arg = {'x0' : [0] * len(self.w0),
        # self.arg = {'x0' : self.w0_initial['x'].full().reshape(-1,).tolist(),
                   'lbx': self.lbw,
                   'ubx': self.ubw,
                   'lbg': self.lbg,
                   'ubg': self.ubg,
                   'p': self.PvX.assemble()}

        return self
    
    def plot_solver_stats(self):
        
        # Plotting stats for the various itreations
        figs, axs = plt.subplots(2, 4)
        stats = self.solver_stats
        from matplotlib.pyplot import cm
        color=cm.YlOrRd(np.linspace(0,1,len(stats)))
        transparency = np.logspace(-1, 0, base=2, num=len(stats))
        for k in range(len(stats)):
            axs[0, 0].plot(stats[k]['iterations']['alpha_du'], c = color[k], alpha = transparency[k], zorder = k)
            axs[0, 0].set_title('alpha_du')
            axs[0, 1].plot(stats[k]['iterations']['alpha_pr'], c = color[k], alpha = transparency[k], zorder = k)
            axs[0, 1].set_title('alpha_pr')
            axs[0, 2].plot(stats[k]['iterations']['d_norm'], c = color[k], alpha = transparency[k], zorder = k)
            axs[0, 2].set_title('d_norm')
            axs[1, 0].plot(stats[k]['iterations']['inf_du'], c = color[k], alpha = transparency[k], zorder = k)
            axs[1, 0].set_title('inf_du')
            axs[1, 1].plot(stats[k]['iterations']['inf_pr'], c = color[k], alpha = transparency[k], zorder = k)
            axs[1, 1].set_title('inf_pr')
            axs[1, 2].plot(stats[k]['iterations']['mu'], c = color[k], alpha = transparency[k], zorder = k)
            axs[1, 2].set_title('mu')
            axs[0, 3].plot(stats[k]['iterations']['obj'], c = color[k], alpha = transparency[k], zorder = k)
            axs[0, 3].set_title('obj')
            axs[1, 3].plot(stats[k]['iterations']['regularization_size'], c = color[k], alpha = transparency[k], zorder = k)
            axs[1, 3].set_title('regularization_size')
        plt.savefig('stage' + str(self.stage) + 'solver_stats_' + str(self.ID) + '.png')
        
        # Plotting single-valued stats
        fig, ax = plt.subplots()
        f = [stats[i]['f'][0] for i in range(len(stats))]
        ax.plot(f)
        ax.set_title('f')
        plt.savefig('stage' + str(self.stage) + 'solver_stats_f_' + str(self.ID) + '.png')
        
        # Plotting lengthy stuff
        figs, axs = plt.subplots(2, 2)
        for k in range(len(stats)):
            axs[0, 0].plot(stats[k]['g'], c = color[k], alpha = transparency[k], zorder = k)
            axs[0, 0].set_title('g')
            axs[0, 1].plot(stats[k]['lam_g'], c = color[k], alpha = transparency[k], zorder = k)
            axs[0, 1].set_title('lam_g')
            axs[1, 0].plot(stats[k]['lam_p'], c = color[k], alpha = transparency[k], zorder = k)
            axs[1, 0].set_title('lam_p')
            axs[1, 1].plot(stats[k]['lam_x'], c = color[k], alpha = transparency[k], zorder = k)
            axs[1, 1].set_title('lam_x')
        plt.savefig('stage' + str(self.stage) + 'solver_stats2_' + str(self.ID) + '.png')
        
        # axs[0, 0].plot(self.solver_stats[0]['iterations']['alpha_du'])
        # plt.savefig(':).png')
        

    def x_update(self):

        self.update_PvX()
        # Updating the necessary arguments for the solver
        self.arg['x0'] = self.DvX.w0
        self.arg['p'] = self.PvX.assemble()
        # Solving the problem
        t1 = time.time()
        self.solution = self.solver.call(self.arg)
        t2 = time.time()
        self.solver_stats += [self.solver.stats()]
        self.solver_stats[-1]['f'] = self.solution['f'].full().reshape(-1,).tolist()
        self.solver_stats[-1]['g'] = self.solution['g'].full().reshape(-1,).tolist()
        self.solver_stats[-1]['lam_g'] = self.solution['lam_g'].full().reshape(-1,).tolist()
        self.solver_stats[-1]['lam_p'] = self.solution['lam_p'].full().reshape(-1,).tolist()
        self.solver_stats[-1]['lam_x'] = self.solution['lam_x'].full().reshape(-1,).tolist()
        self.x_update_time += [t2-t1]
        # Extracting the solution
        self.DvX.extract(self.solution)
        # Saving to history
        self.variable_history["y"] += [self.DvX.y]

        return self


    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    "Simple variable related functions"
    # def define_MX_variable(self, n_dim, lower_bound, upper_bound, name, category):

    #     if category == 'variable':
    #         self.w += [MX.sym(name, n_dim)]
    #         self.w_list += [name for i in range(n_dim)]
    #         self.lbw += [lower_bound] * n_dim
    #         self.ubw += [upper_bound] * n_dim

    #         return self.w[-1]

    #     elif category == 'parameter':
    #         self.P += [MX.sym(name, n_dim)]
    #         self.P_list += [name for i in range(n_dim)]

    #         return self.P[-1]

    #     else:
    #         raise NotImplementedError()



    ###########################################################################
    ###########################################################################
    "Spline related functions"
    def define_knots(self, degree = 3, **kwargs):
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

    def define_MX_spline(self, degree, knot_intervals, n_spl, lower_bound, upper_bound, initial_value = [], name = '', category = 'variable'):
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
            if category == 'variable':
                self.w += [coeffs]
                self.w_list += [name[k] for i in range(len(basis))]
                # if initial_value is not an empty list
                """initial value can be: empty, a list of starting and final values and a list of coefficients
                empty: we initialize it with uniform random variables
                list of starting and final values: we initialize it with np.linspace and adding some noise to it.
                    If any of the starting or final values for the given spline is None, we also initialize it with
                    uniform random variables.
                list of coefficients: we initialize it with the coefficients provided
                """
                if initial_value:
                    if len(initial_value[k]) == len(basis):
                        self.w0 += initial_value[k]
                        # assert True == print("Lefutott LOL")
                    elif initial_value[k][0] is not None or initial_value[k][1] is not None:
                        # self.w0 += np.linspace(initial_value[k][0], initial_value[k][1], len(basis)).tolist()
                        w0_noise_added = np.linspace(initial_value[k][0], initial_value[k][1], len(basis))
                        import random
                        for i in range(len(w0_noise_added)):
                            w0_noise_added[i] = w0_noise_added[i] + w0_noise_added[i] * 0.05 * random.uniform(-0.5, 0.5)

                        self.w0 += w0_noise_added.tolist()
                    else:
                        # self.w0 += np.zeros((1, len(basis)))[0].tolist()
                        import random
                        self.w0 += [random.uniform(-0.5, 0.5) for i in range(len(basis))]

                else:
                    # self.w0 += np.zeros((1, len(basis)))[0].tolist()
                    import random
                    self.w0 += [random.uniform(-0.5, 0.5) for i in range(len(basis))]

            elif category == 'parameter':
                self.P += [coeffs]
                self.P_list += [name[k] for i in range(len(basis))]
                self.P0 += [0 for i in range(len(basis))]
            else:
                raise NotImplementedError()
            splines += [BSpline(basis, coeffs)]


        for i in range(len(splines)):
            for j in range(splines[i].coeffs.shape[0]):
                if category == 'variable':
                    self.lbw += [lower_bound[i]]
                    self.ubw += [upper_bound[i]]

        return np.array(splines)

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
                self.g_list += [name[i]]
                self.lbg += [lower_bound[i]]
                self.ubg += [upper_bound[i]]
            return self

        elif constraint_type == 'final':
            for i in range(len(constraint)):
                self.g += [constraint[i].coeffs[-1]] # we restrict the last coefficient
                self.g_list += [name[i]]
                self.lbg += [lower_bound[i]]
                self.ubg += [upper_bound[i]]
            return self

        else:
            raise NotImplementedError()

    def collision_avoidance_circular(self, splines, center, radious, name):
        """This function defines constraints on the splines to avoid the space arodund
        a certaint point with a given radious.
        Input:
            splines (list): a spline on which we want to set final constraint
            center: the point to avoid
            radious: the minimum distance from the center point
        Returns:

        """

        constraint = (splines[0] - center[0])**2 + (splines[1] - center[1])**2
        self.define_constraint([constraint], lower_bound = [radious**2], upper_bound = [math.inf], name = name)

    def collision_avoidance_hyperplane(self, splines, points, radious, name, constraint_type='obstacle', n_samples = 10):
        """Collision avoidance using the separating hyperplane theorem"""

        # a
        # a points towards the obstacle. This means, that if we use
        # initial_value = [[1, -1], [0.1, 0.1]]
        # the vehicle will try to avoid the obstacle from the bottom direction (because
        # a always points "up" a little bit, it's y coordinate is 0.1)
        a = self.define_MX_spline(degree = 1, knot_intervals = self.knot_intervals, n_spl = len(splines),
                                lower_bound = [-math.inf] * len(splines),
                                upper_bound = [math.inf] * len(splines),
                                initial_value = [[1, -1], [0, 0]],
                                # name = ["a"+ str(i) for i in range(len(splines))])
                                name = ["a"] * self.n_dimensions)

        # b
        b = self.define_MX_spline(degree = 1, knot_intervals = self.knot_intervals, n_spl = 1,
                                lower_bound = [-math.inf],
                                upper_bound = [math.inf],
                                # initial_value = [[0, 0]],
                                name = ["b"])
        # initial_value = [[self.x0[0], self.xf[0]], [self.x0[1], self.xf[1]]],
        # d_tau
        d_tau = self.define_MX_spline(degree = 1, knot_intervals = self.knot_intervals, n_spl = 1,
                                lower_bound = [0],
                                upper_bound = [math.inf],
                                name = ["d_tau"])


        # ---- Constraint 1
        const1 = a[0]*splines[0] + a[1]*splines[1] - b[0]
        self.define_constraint([const1], lower_bound = [-math.inf], upper_bound = [-radious], name = "eq1")

        # ---- Constraint 2 (various versions)
        const2 = []
        if constraint_type == 'obstacle':
            for point in points:
                const2 += [a[0] * point[0] + a[1] * point[1] - b[0]  - d_tau[0] ]
            # ----
            for i in range(len(points)):
                self.define_constraint([const2[i]], lower_bound = [0], upper_bound = [math.inf], name = "eq2" + "_corn_" + str(i))

        elif constraint_type == 'spline_obstacle_t':
            t = np.linspace(0, 1, n_samples)
            for t_ in t:
                for point in points:
                    # const2 += [a[0](t) * point[0](t) + a[1](t) * point[1](t) - b[0](t)]
                    const2 += [a[0](t) * point[0] + a[1](t) * point[1] - b[0](t)  - d_tau[0](t) ]

            # ----
            for i in range(len(const2)):
                self.define_constraint([const2[i]], lower_bound = [0], upper_bound = [math.inf], constraint_type = "time", name = "eq2" + "_corn_" + str(i))


        elif constraint_type == 'inter_vehicle':
            for i in range(points[0].coeffs.shape[0]):
                const2 += [a[0] * points[0].coeffs[i] + a[1] * points[1].coeffs[i] - b[0]  - d_tau[0] ]

            # ----
            for i in range(points[0].coeffs.shape[0]):
                self.define_constraint([const2[i]], lower_bound = [0], upper_bound = [math.inf], name = "eq2" + "_" + str(i))

        elif constraint_type == 'inter_vehicle_t':
            t = np.linspace(0, 1, n_samples)
            for t_ in t:
                const2 += [a[0](t) * points[0](t) + a[1](t) * points[1](t) - b[0](t)  - d_tau[0](t) ]

            # ----
            for i in range(len(const2)):
                self.define_constraint([const2[i]], lower_bound = [0], upper_bound = [math.inf], constraint_type = "time", name = "eq2" + "_" + str(i))
        else:
            raise NotImplementedError()



        # if constraint_type == 'obstacle':

        # elif constraint_type == 'spline_obstacle_t':

        # elif constraint_type == 'inter_vehicle':

        # elif constraint_type == 'inter_vehicle_t':


        # ---- Constraint 3
        const3 = a[0] * a[0] + a[1] * a[1]
        self.define_constraint([const3], lower_bound = [0.0], upper_bound = [1.0], name = "eq3")

        self.J += self.safety_weight * definite_integral((self.epsilon - d_tau[0])**2, 0, 1)

        return self



    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    "Performance related functions"
    def calculate_formation_error(self):
        """
        This function calculates the formation error.
        It is calculated by summing the cross-products of the relevant vectors.

        # Returns:
            formation_error: Array, holds for each ADMM iteration step an array, that containts form errors for each timestep N
                             size is: (ADMM iteration x timestep N)
            formation_error_summed_over_iter: Array, holding formation error for each if the ADMM iterations summed up over N timesteps
                                              size is: (ADMM iteration)
        """
        def cross_product(a, b):
            a1, a2 = a[0], a[1]
            b1, b2 = b[0], b[1]
            return (a1 * b2 - a2 * b1)**2

        hist = self.variable_history
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        # BSpline(basis, coeffs1)
        t = np.linspace(0, 1, 100)
        formation_error = [] # Array, holds for each ADMM iteration step an array, that containts form errors for each timestep N
                             # size is: (ADMM iteration x linstep of t)
        for k in range(len(hist["y"])):
            res = [] # Array, holding the sum of form. errors for all agents for a single linstep of t
                     # size is: (linstep of t)

            # Building y B-splines
            coeffs1 = hist["y"][k][:len(basis)]
            coeffs2 = hist["y"][k][len(basis):]
            y = [BSpline(basis, coeffs1)]
            y += [BSpline(basis, coeffs2)]

            # Building y_j B-splines
            y_j = {}
            for i in range(len(self.neighbours)):
                idx = i * ( self.state_len * len(basis) )
                coeffs1 = hist["y_j"][k][idx + len(basis) * (0) : idx + len(basis) * (1)]
                coeffs2 = hist["y_j"][k][idx + len(basis) * (1) : idx + len(basis) * (2)]
                y_j[str(i)] = [BSpline(basis, coeffs1)]
                y_j[str(i)] += [BSpline(basis, coeffs2)]

            for t_ in t:
                tmp_res = 0
                for j in range(len(self.neighbours)):
                    vec1 = [ (y[0](t_) - y_j[str(j)][0](t_)).tolist()[0],
                             (y[1](t_) - y_j[str(j)][1](t_)).tolist()[0] ]

                    vec2 = [ self.xf[0] - self.neighbours[j].xf[0],  self.xf[1] - self.neighbours[j].xf[1] ]
                    # We can also normalise these two vectors
                    vec1 = (np.array(vec1) / np.linalg.norm(np.array(vec1))).tolist()
                    vec2 = (np.array(vec2) / np.linalg.norm(np.array(vec2))).tolist()

                    tmp_res += cross_product(vec1, vec2)
                res += [tmp_res]
            formation_error += [res]
            formation_error_summed_over_iter = [sum(one_iter) for one_iter in formation_error]

        # plt.close('all')
        # from matplotlib.pyplot import cm
        # color=cm.rainbow(np.linspace(0,1,len(formation_error)))
        # [plt.plot(formation_error[i], c = color[i, :]) for i in range(len(formation_error))]

        # plt.savefig(self.cwd + '/figures/' +'{:0>1d}'.format(self.stage) + 'formation_error.png', dpi = 100)
        # plt.figure()
        # plt.plot(formation_error_summed_over_iter)

        # plt.savefig(self.cwd + '/figures/' +'{:0>1d}'.format(self.stage) + 'formation_error_summed_over_iter.png', dpi = 100)
        return formation_error, formation_error_summed_over_iter
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    ###########################################################################
    "Plotting"
    def get_final_trajectories_t(self):
        flatten = lambda t: [item for sublist in t for item in sublist]
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)

        solution = self.solution['x'].full()
        coeffs1 = flatten([solution[x] for x in np.arange(0, len(basis))])
        coeffs2 = flatten([solution[x] for x in np.arange(len(basis), len(basis)*2)])

        x = BSpline(basis, coeffs1)
        y = BSpline(basis, coeffs2)

        # Sampling
        t = np.linspace(0, 1, 100)
        x_t = [x(t_) for t_ in t]
        y_t = [y(t_) for t_ in t]

        x_t = flatten(x_t)
        y_t = flatten(y_t)

        return x_t, y_t


    def make_spline(self, coeffs):
        """ This function genereates a spline using the provided coefficients (assuming,
        that the degree & the number of knot intervals is default)
        """
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        spline = BSpline(basis, coeffs)
        return spline

    def plot_vehicle_trajectories_gradient(self, ax):
        # Plotting the environment
        ax = self.plot_environment(ax)
        # Plotting of obstacles
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(ax)

        # Plotting trajectories of previous iterations
        hist = self.variable_history
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        from matplotlib.pyplot import cm
        color=cm.Wistia(np.linspace(0,1,len(hist["y"])))
        color=cm.YlOrRd(np.linspace(0,1,len(hist["y"])))
        t = np.linspace(0, 1, 100)
        # color = [(0.0, 0.0, 0.0, linspace_) for linspace_ in np.linspace(0, 1, len(hist["y"])) ]
        # color = [(0.0, 0.0, 0.0, linspace_) for linspace_ in np.logspace(-9, -5, base=2, num=len(hist["y"])) ]
        transparency = np.logspace(-9, -5, base=2, num=len(hist["y"]))
        transparency = np.logspace(-1, 0, base=2, num=len(hist["y"]))
        for k in range(len(hist["y"])):
            coeffs1 = hist["y"][k][:len(basis)]
            coeffs2 = hist["y"][k][len(basis):]
            y = [BSpline(basis, coeffs1)]
            y += [BSpline(basis, coeffs2)]
            yx = [y[0](t_).reshape(1,).tolist()[0] for t_ in t]
            yy = [y[1](t_).reshape(1,).tolist()[0] for t_ in t]
            ax.plot(yx, yy, c = color[k], alpha = transparency[k], zorder = k)
        # The last should be pure black
        ax.plot(yx, yy, 'k', zorder = k+1)

        return ax

    def plot_vehicle_trajectories(self, ax):
        flatten = lambda t: [item for sublist in t for item in sublist]

        # Plotting the environment
        ax = self.plot_environment(ax)

        # Creating the splines
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        try:
            solution = self.solution['x'].full()
        except:
            solution = self.w0_initial['x'].full()
        coeffs1 = flatten([solution[x] for x in np.arange(0, len(basis))])
        coeffs2 = flatten([solution[x] for x in np.arange(len(basis), len(basis)*2)])

        x = BSpline(basis, coeffs1)
        y = BSpline(basis, coeffs2)

        # Sampling
        t = np.linspace(0, 1, 100)
        x_t = [x(t_) for t_ in t]
        y_t = [y(t_) for t_ in t]

        x_t = flatten(x_t)
        y_t = flatten(y_t)

        # Fitting a polynome of degree 7 onto the spline
        poly7_x = np.poly1d(np.polyfit(t, x_t, deg=7))
        poly7_y = np.poly1d(np.polyfit(t, y_t, deg=7))

        # Now, the 7 degree polynomials are calculated such that upon evaluation
        # between t = [0, 1] we get correct values. Outside this range, they don't
        # represent the trajectories we calculated.
        # Below we rescale the polynomials such that they give correct values
        # between t = [0, t_desired]
        # Remember: x^7 --> x^7 / t_desired^7
        power = 0
        for i in range(len(poly7_x.coeffs)-1, 0-1, -1):
            poly7_x.coeffs[i] = poly7_x.coeffs[i] / pow(self.T, power)
            poly7_y.coeffs[i] = poly7_y.coeffs[i] / pow(self.T, power)
            power += 1

        # Let's now sample from this, for the sake of plotting
        tp7 = np.linspace(0, self.T, 100)
        poly7_x_t = [poly7_x(t_) for t_ in tp7]
        poly7_y_t = [poly7_y(t_) for t_ in tp7]

        # The path !!! now with correct arrangement of the coefficients !!!
        # Storing the coefficients in the format, that crazyswarm requires
        # (x^0, x^1, x^2, ...)
        poly7_x = poly7_x.coeffs.tolist()
        poly7_x.reverse()
        poly7_y = poly7_y.coeffs.tolist()
        poly7_y.reverse()
        # self.write_csv(self.T, poly7_x, poly7_y)

        # Adding an extra 7 degree polynomial for hoowering at the end
        final_x = poly7_x_t[-1]
        final_y = poly7_y_t[-1]
        however_x = [final_x] + [0] * 7
        however_y = [final_y] + [0] * 7

        # Combining the polinomials into a list
        T_list = [[self.T], [2]]
        poly7_x_list = [poly7_x, however_x]
        poly7_y_list = [poly7_y, however_y]

        # Writing the list to file
        self.write_csv(T_list, poly7_x_list, poly7_y_list)

        ax.plot(poly7_x_t, poly7_y_t, 'k*')

        # Plotting of spline & control points
        ax.plot(coeffs1, coeffs2, 'ro')
        ax.plot(x_t, y_t, '.')
        

        # Plotting of obstacle
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(ax)

        return ax

    def save_trajectory_to_csv(self, t_desired = 5, t_hover = 2):
        """ This function is pretty much doing the same as plot_vehicle_trajectories() + write_csv()
        """
        flatten = lambda t: [item for sublist in t for item in sublist]

        hist = self.variable_history

        # Creating the splines
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)

        coeffs1 = hist["y"][-1][:len(basis)]
        coeffs2 = hist["y"][-1][len(basis):]
        x = BSpline(basis, coeffs1)
        y = BSpline(basis, coeffs2)

        # Sampling
        t = np.linspace(0, 1, 100)
        x_t = [x(t_) for t_ in t]
        y_t = [y(t_) for t_ in t]

        x_t = flatten(x_t)
        y_t = flatten(y_t)

        # Fitting a polynome of degree 7 onto the spline
        poly7_x = np.poly1d(np.polyfit(t, x_t, deg=7))
        poly7_y = np.poly1d(np.polyfit(t, y_t, deg=7))


        # Now, the 7 degree polynomials are calculated such that upon evaluation
        # between t = [0, 1] we get correct values. Outside this range, they don't
        # represent the trajectories we calculated.
        # Below we rescale the polynomials such that they give correct values
        # between t = [0, t_desired]
        # Remember: x^7 --> x^7 / t_desired^7
        power = 0
        for i in range(len(poly7_x.coeffs)-1, 0-1, -1):
            poly7_x.coeffs[i] = poly7_x.coeffs[i] / pow(t_desired, power)
            poly7_y.coeffs[i] = poly7_y.coeffs[i] / pow(t_desired, power)
            power += 1

        # Adding an extra 7 degree polynomial for hoowering at the end
        final_x = poly7_x(t_desired)
        final_y = poly7_y(t_desired)
        however_x = [final_x] + [0] * 7
        however_y = [final_y] + [0] * 7

        # The path !!! now with correct arrangement of the coefficients !!!
        # Storing the coefficients in the format, that crazyswarm requires
        # (x^0, x^1, x^2, ...)
        poly7_x = poly7_x.coeffs.tolist()
        poly7_x.reverse()
        poly7_y = poly7_y.coeffs.tolist()
        poly7_y.reverse()

        # Combining the polinomials into a list
        T_list = [[t_desired], [t_hover]]
        poly7_x_list = [poly7_x, however_x]
        poly7_y_list = [poly7_y, however_y]

        # Writing the list to file
        self.write_csv(T_list, poly7_x_list, poly7_y_list)
        self.write_csv_for_stage(T_list, poly7_x_list, poly7_y_list)

        return self
    
    def write_csv_for_stage(self, T, px, py):
        """This function writes a csv for only the specific stage.
        It won't include any other stages, but can be used later to assemble
        trajectories for all stages."""
        
        pz = [0] * len(px[0])
        pj = [0] * len(px[0])
        first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']
        mode = 'w'
        with open(self.cwd + '/csv/' + 'stage_' + str(self.stage) + '_vehicle' + str(self.ID) + '.csv', mode = mode) as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(first_line)
            for i in range(len(T)):
                writer.writerow(T[i] + px[i] + py[i] + pz + pj)
                
        return self
    
    def write_csv(self, T, px, py):

        # px, py, pz, pj = [0], [0], [0], [0]

        pz = [0] * len(px[0])
        pj = [0] * len(px[0])

        # Writing this first line is required by crazyswarm
        first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']

        # If we are at the first stage, we will create a new csv file
        append_row = []
        if self.stage == 0:
            mode = 'w'
        # If we have already completed the second stage, we will add new rows to the csv file.
        # For this reason, first we:
        # 1: read in the lines.
        # 2: store them in append_row and swith back to writing mode
        else:
            mode = 'r'
            with open(self.cwd + '/csv/' + 'vehicle' + str(self.ID) + '.csv', mode = mode) as csvfile:
                reader = csv.reader(csvfile)
                for row in reader:
                    append_row += [row]
            mode = 'w'

        with open(self.cwd + '/csv/' + 'vehicle' + str(self.ID) + '.csv', mode = mode) as csvfile:
            writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        # 3: write the first line
            if self.stage == 0:
                writer.writerow(first_line)
        # 4: OR write the lines associated with previous stages in apend-row
        # (basically except the last two lines)
            for i, row in enumerate(append_row):
                if i < self.stage*2 + 1:
                    writer.writerow(row)
        # 5: because instead of the last two lines we will write
        # the polynome values belonging to the most recent solution.
            for i in range(len(T)):
                writer.writerow(T[i] + px[i] + py[i] + pz + pj)
        return self

    # def read_csv(self):

    #     import pandas as pd
    #     group_trajectories = {}
    #     for vehicle_num in range(3):
    #         trajectories = pd.read_csv('vehicle' + str(vehicle_num) + '.csv')
    #         trajectories = np.array(trajectories)
    #         x, y, z = [], [], []
    #         for i in range(trajectories.shape[0]):
    #             x_polynom_coeffs = trajectories[i, 1+8*0:1+8*1].tolist()
    #             x_polynom_coeffs.reverse()
    #             y_polynom_coeffs = trajectories[i, 1+8*1:1+8*2].tolist()
    #             y_polynom_coeffs.reverse()
    #             z_polynom_coeffs = trajectories[i, 1+8*2:1+8*3].tolist()
    #             z_polynom_coeffs.reverse()

    #             # x = np.poly1d(x_polynom_coeffs)
    #             # y = np.poly1d(y_polynom_coeffs)
    #             # z = np.poly1d(z_polynom_coeffs)

    #             # print(x)
    #             # print(y)
    #             # print(z)

    #             x += [np.poly1d(x_polynom_coeffs)]
    #             y += [np.poly1d(y_polynom_coeffs)]
    #             z += [np.poly1d(z_polynom_coeffs)]
    #         group_trajectories[str(vehicle_num)] = {'x' : x, 'y' : y, 'z' : z}



    def plot_rotation(self, x, y, theta):
        x_new = x * cos(theta) - y * sin(theta)
        y_new = x * sin(theta) + y * cos(theta)
        return x_new, y_new

    def plot_path_frames(self, ax, t):
        # Plotting trajectories of previous iterations
        hist = self.variable_history
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        coeffs1 = hist["y"][-1][:len(basis)]
        coeffs2 = hist["y"][-1][len(basis):]
        yx = BSpline(basis, coeffs1)
        yy = BSpline(basis, coeffs2)
        t_grey = np.linspace(0, t, 100)
        t_black = np.linspace(t, 1, 100)
        # Already covered path
        grey_x = [yx(t_grey_).tolist()[0] for t_grey_ in t_grey]
        grey_y = [yy(t_grey_).tolist()[0] for t_grey_ in t_grey]
        # Remaining path
        black_x = [yx(t_black_).tolist()[0] for t_black_ in t_black]
        black_y = [yy(t_black_).tolist()[0] for t_black_ in t_black]

        # Changing up the colors :DDD
        # ax.plot(grey_x, grey_y, c = [0, 0, 0, 1])
        # ax.plot(black_x, black_y, c = [0, 0, 0, 0.2])
        ax.plot(grey_x, grey_y, c = 'cornflowerblue',lw=0.8,alpha = 1, zorder = 3)
        ax.plot(black_x, black_y, c = 'cornflowerblue',lw=0.8,alpha = 0.5, zorder = 5)
        
        # Plotting GUROBI path
        # ax.plot(self.gurobi_x, self.gurobi_y, ':')
        # ax.plot(self.gurobi_x, self.gurobi_y, 'k.')
        # Plotting ASTAR path
        try:
            ax.plot(self.astar_x, self.astar_y, 'k:')
            ax.plot(self.astar_x, self.astar_y, 'b.')
        except:
            print("Warning - No astar path found")
        # Plotting BSpline coeffs
        ax.plot(coeffs1, coeffs2, 'r.')
        
        
        return ax

    def plot_moovie_frames(self, ax, t):
        # https://nickcharlton.net/posts/drawing-animating-shapes-matplotlib.html

        flatten = lambda t: [item for sublist in t for item in sublist]

        # Plotting the environment
        ax = self.plot_environment(ax)

        # Plotting of obstacle
        for obstacle in self.obstacles:
            obstacle.plot_obstacle(ax)



        # Creating the splines
        basis = self.define_knots(degree = self.state_degree, knot_intervals = self.knot_intervals)
        solution = self.solution['x'].full()
        coeffs1 = flatten([solution[x] for x in np.arange(0, len(basis))])
        coeffs2 = flatten([solution[x] for x in np.arange(len(basis), len(basis)*2)])

        x = BSpline(basis, coeffs1)
        y = BSpline(basis, coeffs2)

        theta_c = 0

        # Draw body
        # radious = 0.05
        radious = self.radious * 1 * 0.9
        # height = radious
        # width = radious
        height = 0.03
        width = 0.03


        # Draw arms
        # l = 0.1
        # l = radious * 2
        l = 0.095 / 2


        x0 = x(t)
        y0 = y(t)



        rot_x, rot_y = self.plot_rotation(l, 0, theta_c + np.pi/4)
        x1 = x0 + rot_x
        y1 = y0 + rot_y

        rot_x, rot_y = self.plot_rotation(l, 0, theta_c + np.pi/4*3)
        x2 = x0 + rot_x
        y2 = y0 + rot_y

        rot_x, rot_y = self.plot_rotation(l, 0, theta_c + np.pi/4*5)
        x3 = x0 + rot_x
        y3 = y0 + rot_y

        rot_x, rot_y = self.plot_rotation(l, 0, theta_c + np.pi/4*7)
        x4 = x0 + rot_x
        y4 = y0 + rot_y


        # Rotors
        # https://stackoverflow.com/questions/9215658/plot-a-circle-with-pyplot
        # r_rotor = 0.03
        # r_rotor = l * 0.3
        r_rotor = 0.045 / 2

        # Adding stuff to ax

        # Lines
        line1 = Line2D([x0, x1], [y0, y1], zorder = 8)
        line2 = Line2D([x0, x2], [y0, y2], zorder = 8)
        line3 = Line2D([x0, x3], [y0, y3], zorder = 8)
        line4 = Line2D([x0, x4], [y0, y4], zorder = 8)
        ax.add_line(line1)
        ax.add_line(line2)
        ax.add_line(line3)
        ax.add_line(line4)

        # Circles
        circle1   = plt.Circle((x1, y1), r_rotor*1.3, fc=[0.2, 0.2, 0.2, 1], ec=None,                alpha=0.6, fill=True,  zorder = 10)
        circle1_2 = plt.Circle((x1, y1), r_rotor*0.8, fc=None,               ec=[0, 0, 0, 1],lw=0.4, alpha=0.5, fill=False, zorder = 10)
        circle2   = plt.Circle((x2, y2), r_rotor*1.3, fc=[0.2, 0.2, 0.2, 1], ec=None,                alpha=0.6, fill=True,  zorder = 10)
        circle2_2 = plt.Circle((x2, y2), r_rotor*0.8, fc=None,               ec=[0, 0, 0, 1],lw=0.4, alpha=0.5, fill=False, zorder = 10)
        circle3   = plt.Circle((x3, y3), r_rotor*1.3, fc=[0.2, 0.2, 0.2, 1], ec=None,                alpha=0.6, fill=True,  zorder = 10)
        circle3_2 = plt.Circle((x3, y3), r_rotor*0.8, fc=None,               ec=[0, 0, 0, 1],lw=0.4, alpha=0.5, fill=False, zorder = 10)
        circle4   = plt.Circle((x1, y4), r_rotor*1.3, fc=[0.2, 0.2, 0.2, 1], ec=None,                alpha=0.6, fill=True,  zorder = 10)
        circle4_2 = plt.Circle((x4, y4), r_rotor*0.8, fc=None,               ec=[0, 0, 0, 1],lw=0.4, alpha=0.5, fill=False, zorder = 10)
        # circle2 = plt.Circle((x2, y2), r_rotor, color='k', alpha=0.5, zorder = 10)
        # circle3 = plt.Circle((x3, y3), r_rotor, color='k', alpha=0.5, zorder = 10)
        # circle4 = plt.Circle((x4, y4), r_rotor, color='k', alpha=0.5, zorder = 10)
        ax.add_patch(circle1)
        ax.add_patch(circle1_2)
        ax.add_patch(circle2)
        ax.add_patch(circle2_2)
        ax.add_patch(circle3)
        ax.add_patch(circle3_2)
        ax.add_patch(circle4)
        ax.add_patch(circle4_2)
        # ax.add_patch(circle2)
        # ax.add_patch(circle3)
        # ax.add_patch(circle4)




        rect_x = - width/2
        rect_y = - height/2
        rot_x, rot_y = self.plot_rotation(rect_x, rect_y, theta_c)

        rectangle = patches.Rectangle((x0 + rot_x, y0 + rot_y), height, width,
                                      linewidth=1, edgecolor='k', facecolor='k',
                                      zorder = 9, angle = theta_c / np.pi * 180)
        ax.add_patch(rectangle)



        return ax
