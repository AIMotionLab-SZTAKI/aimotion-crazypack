from matplotlib.patches import Polygon
import numpy as np
import os

class Environment():
    def __init__(self):
        
        # self.border_x = [-1.5 , 1.5]
        # self.border_y = [-0.8, 0.8]
        self.border_x = [-2, 2]
        self.border_y = [-1, 1]
        
        self.obstacle_area = { 'x_limits': [-0.2, 0.6], 'y_limits': [-0.5, 0.5] }
        self.obstacle_avoidance_multiplier = 1.5
        self.vehicle_avoidnce_multiplier = 2.0
        
        self.cwd = os.getcwd()
        
        
        
    def plot_environment(self, ax):
        # Plotting border
        corners = [[self.border_x[0], self.border_y[0]],
                   [self.border_x[1], self.border_y[0]],
                   [self.border_x[1], self.border_y[1]],
                   [self.border_x[0], self.border_y[1]],
                   [self.border_x[0], self.border_y[0]]
                   ]
        for obstacle in self.obstacles:
            corners = np.array(corners)
            polygon = Polygon(corners, closed=True, fill=False,
                              linestyle = '--', 
                              fc=(0,0,0,0.1), ec=(0,0,0,1), lw=1, zorder = 1)
            ax.add_patch(polygon)
            
        return ax
