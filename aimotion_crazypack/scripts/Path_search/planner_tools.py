import sys
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import math
import networkx as nx
from queue import PriorityQueue
from scipy import interpolate
from scipy import linalg
from scipy.spatial import Delaunay
from shapely.geometry import Point, Polygon, LineString
from gurobipy import *
import csv
import copy

def write_initial_position_yaml(drone1_x0, drone2_x0):
    # Writing initial position
    import yaml
    initialPosition = drone1_x0
    targetHeight = 0.8
    initialPosition = initialPosition + [float(targetHeight)]
    yaml_dict = {'crazyflies' : []}
    yaml_dict['crazyflies'] += [{'id' : 0, 'channel' : 100,
                                 'initialPosition' : initialPosition,
                                 'type' : 'default'
                                 }]
    
    initialPosition = drone2_x0
    targetHeight = 0.8
    initialPosition = initialPosition + [float(targetHeight)]
    yaml_dict['crazyflies'] += [{'id' : 1, 'channel' : 100,
                             'initialPosition' : initialPosition,
                             'type' : 'default'
                             }]
        
    with open("../dijkstra/yaml/initialPosition.yaml", "w") as file_descriptor:
                    yaml.dump(yaml_dict, file_descriptor)

def random_V0_generator(x_lim, y_lim, d_min, p0 = None, d_max = math.inf, p2_clear = np.array([math.inf, math.inf]) ):
    "This function...."
    
    
    # We need the positions to be at least d_min apart
    d_points = 0
    d_clear = 0.4
    d2_clear = 0
    while not(d_min <= d_points <= d_max) or not(d_clear <= d2_clear):
        if p0 == None:
            x_rand1 = np.random.rand() * (x_lim[1] - x_lim[0]) - (x_lim[1] - x_lim[0]) / 2
            y_rand1 = np.random.rand() * (y_lim[1] - y_lim[0]) - (y_lim[1] - y_lim[0]) / 2
        else:
            x_rand1 = p0[0]
            y_rand1 = p0[1]
            
        x_rand2 = np.random.rand() * (x_lim[1] - x_lim[0]) - (x_lim[1] - x_lim[0]) / 2
        y_rand2 = np.random.rand() * (y_lim[1] - y_lim[0]) - (y_lim[1] - y_lim[0]) / 2
        
        p_1 = np.array([x_rand1, y_rand1])
        p_2 = np.array([x_rand2, y_rand2])
        
        d_points = np.linalg.norm(p_1 - p_2)
        d2_clear = np.linalg.norm(p_2 - p2_clear)
    
    return p_1.tolist(), p_2.tolist()
        
def generating_box_urdf(position, ID):

    urdf_part1 = """<?xml version="0.0" ?>
<robot name="cube.urdf">
  <link name="baseLink">
    <contact>
      <lateral_friction value="1.0"/>
      <rolling_friction value="1.0"/>
      <inertia_scaling value="3.0"/>
      <contact_cfm value="0.0"/>
      <contact_erp value="1.0"/>
    </contact>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
       <mass value=".0"/>
       <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin rpy="1.5708 0 0" xyz=" """

    urdf_part2 = """ "/>
      <geometry>
				<mesh filename="pole.obj" scale=" """
    urdf_part3 = """ "/>
      </geometry>
       <material name="white">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
	 	<mesh filename="pole.obj" scale="0 0 0"/>
      </geometry>
    </collision>
  </link>
</robot>
"""

    # Calculating center
    center = position
    scale = 1

    # Assembling .urdf file
    urdf_text = urdf_part1 + str(center[0]) + " " + str(center[1]) + " " + str(center[2])  \
    + urdf_part2 + str(scale) + " " + str(scale) + " " + str(scale)  \
    + urdf_part3

    # Writing .urdf file
    filename = "dijkstra/urdf/" + "pole" + str(ID) + ".urdf"
    f = open(filename, "w")
    f.write(urdf_text)
    f.close()
        
    return 0

def generating_traj_csv(object_instance, object_name : str = "vehicle", object_id : int = 0, file_idx = None):
    t_start = object_instance[object_id].path[0][0]
    if object_name == "vehicle":
        t_end = object_instance[object_id].gt
    else:
        t_end = object_instance[object_id].path[0][-1]
    n_divisions = 5
    t_divisions = np.linspace(t_start, t_end, n_divisions + 1)
    poly7_x_slices, poly7_y_slices, t_slices = [], [], []
    for i in range(1, len(t_divisions)):
        t_slice_start = t_divisions[i - 1]
        t_slice_end = t_divisions[i]
        t_slice = np.linspace(t_slice_start, t_slice_end)
        t_slices.append(copy.deepcopy(t_slice))
        
        # x, y = interpolate.splev(drone_obj[0].gurobi_move(frame_number * Ts * 0.001), drone_obj[0].path)
        # patch[-2].center = (x, y)
        # for i in range(0, len(mobs_obj)):
        # x, y = interpolate.splev(mobs_obj[i].move(frame_number * Ts * 0.001), mobs_obj[i].path)
            
        if getattr(type(object_instance[object_id]), '__name__', '') == 'Mobs':
            # x_slice, y_slice = interpolate.splev(object_instance[object_id].move(t_slice), object_instance[object_id].path)
            x_slice, y_slice = [], []
            try:
                for t_slice_ in t_slice:
                    tmp_x, tmp_y = interpolate.splev(object_instance[object_id].move(t_slice_), object_instance[object_id].path)
                    x_slice.append(tmp_x)
                    y_slice.append(tmp_y)
            except:
                pass
        elif getattr(type(object_instance[object_id]), '__name__', '') == 'Drone':
            x_slice, y_slice = [], []
            for t_slice_ in t_slice:
                tmp_x, tmp_y = interpolate.splev(object_instance[object_id].gurobi_move(t_slice_), object_instance[object_id].path)
                x_slice.append(tmp_x)
                y_slice.append(tmp_y)
        else:
            raise NotImplementedError()
            
        # x_slice, y_slice = interpolate.splev(t_slice, object_instance[object_id].path)
        
        # Let us correct for the fact, that the crazyflies will evaulate each 
        # polynomial piece between t \in [0, t_desired]
        t_shift = t_slices[i-1][0]
        t_slices[i-1] -= t_shift
        
        poly7_x_slice = np.poly1d(np.polyfit(t_slices[i-1], x_slice, deg=7))
        poly7_y_slice = np.poly1d(np.polyfit(t_slices[i-1], y_slice, deg=7))
        
        poly7_x_slices.append(poly7_x_slice)
        poly7_y_slices.append(poly7_y_slice)
        
        
    # Let's check, if we get something useful back
    for i in range(len(t_slices)):
        x = poly7_x_slices[i](t_slices[i])
        y = poly7_y_slices[i](t_slices[i])
        plt.plot(x, y, 'k')
        
        
    # Let us slow it down 5 fold
    slow_down = 5
    for slice_count in range(len(poly7_x_slices)):
        power = 0
        for i in range(len(poly7_x_slices[slice_count].coeffs)-1, 0-1, -1):
            poly7_x_slices[slice_count].coeffs[i] = poly7_x_slices[slice_count].coeffs[i] / pow(slow_down, power)
            poly7_y_slices[slice_count].coeffs[i] = poly7_y_slices[slice_count].coeffs[i] / pow(slow_down, power)
            power += 1
        
        
    # Let us write the whole thing into a csv file.
    
    # First, we need to revert the order of the coefficients, because the 
    # Crazyflie wants it the other way around
    poly7_x_coeffs, poly7_y_coeffs, t_desired = [], [], []
    for i in range(len(t_slices)):
        poly7_x_coeffs.append(poly7_x_slices[i].coeffs.tolist()[::-1])
        poly7_y_coeffs.append(poly7_y_slices[i].coeffs.tolist()[::-1])
        t_desired.append(t_slices[i][-1] * slow_down)
        
    # We should hover at the end
    # if getattr(type(object_instance[object_id]), '__name__', '') == 'Drone':
    if object_name == 'vehicle':
        if getattr(type(object_instance[object_id]), '__name__', '') == 'Drone': 
            hower_x, hower_y = interpolate.splev(object_instance[object_id].gurobi_move(object_instance[object_id].gt), object_instance[object_id].path)
        else:
            hower_x, hower_y = interpolate.splev(object_instance[object_id].move(object_instance[object_id].gt), object_instance[object_id].path)
        hower_zeros = [0] * 7
        poly7_x_coeffs.append([hower_x.tolist()] + hower_zeros)
        poly7_y_coeffs.append([hower_y.tolist()] + hower_zeros)
        t_desired.append(0.01)
        
    
        
        
    if file_idx == None:
        write_csv(t_desired, poly7_x_coeffs, poly7_y_coeffs, object_name + str(object_id))
    else:
        write_csv(t_desired, poly7_x_coeffs, poly7_y_coeffs, object_name + str(file_idx))
    
    
    return 0
        

def write_csv(T, px, py, name):

    # px, py, pz, pj = [0], [0], [0], [0]

    pz = [0] * len(px[0])
    pj = [0] * len(px[0])

    # Writing this first line is required by crazyswarm
    first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']
    mode = 'w'
    with open('dijkstra/csv/' + name + '.csv', mode = mode) as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(first_line)
        for i in range(len(T)):
            writer.writerow([T[i]] + px[i] + py[i] + pz + pj)
    return 0
    
#-----------------------------------------------------------------------------------
class Results:                              # We use it to store base data.
    def __init__(self):
        self.s = []
        self.v = []
        self.a = []
        self.tgrid = []
#----------------------------------------------------------------------------------------------------------------------


class Scene:                              # We use it to store base data.
    def __init__(self):
        self.dims = []
        self.fobs = []
        self.box = []
        self.mobs = []
#=======================================================================================================================


class Mobs:                               # We use it to store the data and functions required for the animation.
    def __init__(self, radius, velocity, path, path_length, mode):
        self.r = radius
        self.v = velocity
        self.path = path
        self.len = path_length
        self.mode = mode
        self.gp = []
        self.gt = []

    def move(self, t):                    # Define the types of movements
        if self.mode == "res":            # If the end of the path is reached restart from the start
            s = np.remainder(self.v * t, self.len)
            return s
        elif self.mode == "bnf":          # If the end/start of the path is reached switch direction
            s = back_n_forth(self.v * t, self.len)
            return s
        elif self.mode == "gurobi":
            s = []
            # if type(t).__name__ != 'float64':
            try:
                for t_ in t:
                    s.append(interpolate.splev(min(t_, self.gt), self.gp).tolist())
                return np.array(s)
            except:
            # else:
                s = interpolate.splev(min(t, self.gt), self.gp).tolist()
                return np.array(s)
        else:
            print("ERROR: Mobs.mode  (req: 'res' or 'bnf')")
#.......................................................................................................................


def back_n_forth(t, act_len):             # Helping function to create the back and forth motion
    d = np.floor_divide(t, act_len)
    r = t - d * act_len
    odd = np.remainder(d, 2)
    tt = np.multiply((1 - odd), r) + np.multiply(odd, (act_len - r))
    return tt
#=======================================================================================================================


class Drone:                               # We use it to store the data and functions required for the animation.
    def __init__(self, radius, velocity, path, path_length, gurobi_path, gurobi_time):
        self.r = radius
        self.v = velocity
        self.v_max = 7 * 2
        self.a_max = 5 * 2
        self.path = path
        self.len = path_length
        self.gp = gurobi_path
        self.gt = gurobi_time

    def move(self, t):                    # Define the types of movements
        s = np.remainder(self.v * min(t, self.gt), self.len)
        return s

    def gurobi_move(self, t):
        sg = interpolate.splev(min(t, self.gt), self.gp)
        return sg
#=======================================================================================================================


def create_view(scene, vis_mobs):         # Create the scene and show the mob paths if required
    plt.axis(scene.dims)
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(True)
    plt.xlabel('x')
    plt.ylabel('y')
    dim = len(scene.dims) / 2
    fig = plt.gcf()  # Catch the current fig which previously created above
    ax = fig.gca()

    if len(scene.fobs):                   # Plotting the fix-obstacles if there is any
        if dim == 2:                      # Preparation for 3D variation which is TODO: 3D
            C = scene.fobs[:, 0:2]        # Separate the koordinates of the fix-obstacles
            r = scene.fobs[:, -1]         # Separate the radii of the fix-obstacles
            for i in range(0, C.shape[0]):# Draw the fix-obstacles
                ax.add_artist(plt.Circle((C[i][0], C[i][1]), r[i], color='darkred'))

    if len(scene.box):                    # Plotting the fix-obstacles if there is any
        if dim == 2:                      # Preparation for 3D variation which is TODO: 3D
            for i in range(scene.box.shape[0]):
                x = scene.box[i][:, 0]    # Separate the koordinates of the fix-obstacles
                y = scene.box[i][:, 1]
                for j in range(len(x)):   # Draw the fix-obstacles
                    plt.fill(x, y, color='darkred')

    if len(scene.mobs):                   # Plotting all existing or required moving-obstacle paths
        j = 0
        for i in scene.mobs:
            if j in vis_mobs or 0 == len(vis_mobs):
                path = Path_creator(i, "norm")
                X_path, Y_path = path
                plt.plot(X_path, Y_path, 'blue', lw=1.5, alpha=1)
            j = j+1
#=======================================================================================================================


def Path_creator(xy, mode):
    x = []
    y = []
    len_xy = xy.size / 2                  # Preparing data to set the degree of the splines

    if xy[0, 0] == xy[-1, 0] and xy[0, 1] == xy[-1, 1]:
        Closed_path = True                # Check if a path is closed or not
    else:
        Closed_path = False

    for point in xy:                      # Separate the x y coordinates
        x.append(point[0])
        y.append(point[1])
        
    # Szilárd
    x = xy[:, 0].tolist()
    y = xy[:, 1].tolist()

    if Closed_path:                       # Find the B-spline representation of the xy points
        tck, *rest = interpolate.splprep([x, y], k=3, s=0)  # Closed path
    elif len_xy == 2:
        tck, *rest = interpolate.splprep([x, y], k=1, s=0)  # Line
    else:
        tck, *rest = interpolate.splprep([x, y], k=2, s=0)  # Curve

    u = np.arange(0, 1.01, 0.01)          # Set the breaks of the path
    path = interpolate.splev(u, tck)      # Make the path (x,y point sequence)

    if mode == "norm":                    # In normal mode just return this path
        return path
    elif mode == "csnp":
        X_path, Y_path = path             # Separate the x and y coordinates ('dense' in the original matlab code)
        D = np.diff(path)                 # Calculate the discrete differences between the points
        length = [0, np.cumsum(math.sqrt(np.sum(np.multiply(D, D))))*10]  # Get the length of the path
        par = np.transpose(np.linspace(length[0], length[-1], 101))       # Create length accurate breaks
        par = np.reshape(par, par.size)   # convert the 2D array to a 1D array
        tck, u, *rest = interpolate.splprep([X_path, Y_path], k=2, s=0, u=par)
        return tck, u                     # Find the B-spline representation with accurate break lengths
    else:
        print("ERROR: Path_creator.mode (req: 'norm' or 'csnp')")
#=======================================================================================================================


def tellme(s):                            # Helping function to write out instructions for the making the paths
    print(s)
    plt.title(s, fontsize=16)
    plt.draw()
#=======================================================================================================================


def get_path(xy_list):                    # Take a point sequence by mouse clicks from the scene
    while True:
        xy = []
        while len(xy) < 2:                # Make sure that the user gives at least 2 points
            tellme('Select min 2 points with left mouse click\n'
                   'Stop adding points with right mouse click')
            xy = np.array(plt.ginput(-1, timeout=-1, mouse_stop=3))  # Get the point sequence of the clicks

        if 0.5 >= (math.sqrt(math.pow(xy[0, 0] - xy[-1, 0], 2) + math.pow(xy[0, 1] - xy[-1, 1], 2))):
            xy[-1, 0] = xy[0, 0]          # If the start and end points of the line are close make it a continuous
            xy[-1, 1] = xy[0, 1]            # by connecting the ends

        path = Path_creator(xy, "norm")   # Make a path from the points
        X_path, Y_path = path             # Separate the x and y coordinates
        plt.plot(X_path, Y_path, 'lime', lw=2)  # Draw the path to the scene
        ph = plt.plot(xy[:, 0], xy[:, 1], "o")  # Draw the collected points to the scene
        xy_list += [xy]                   # Add the points to the xy_list

        tellme('Add more line with mouse press\nExit with a button press')
        if plt.waitforbuttonpress():      # If a button is pressed after a path is created end collecting paths
            break

        for p in ph:                      # Remove the points from the scene
            p.remove()

    return xy_list
#=======================================================================================================================


def select_SE(V0, fobs, box):             # Take a point sequence by mouse clicks from the scene
    fig = plt.gcf()                       # Catch the current fig which previously created by create_view
    ax = fig.gca()

    Str = []
    while inside(Str, fobs, box):         # Not accepts points in an obstacle
        tellme('Select the Starting position of the 1st drone')
        Str = np.array(plt.ginput(1, timeout=-1))
    ax.scatter(Str[0][0], Str[0][1], s=10, facecolors='red')
    End = []
    while inside(End, fobs, box):
        tellme('Select the Ending position of the 1st drone')
        End = np.array(plt.ginput(1, timeout=-1))
    ax.scatter(End[0][0], End[0][1], s=10, facecolors='red')

    V0 = np.append(V0, Str, axis=0)
    V0 = np.append(V0, End, axis=0)
    
    # Receiving the second drone's position
    Str2 = []
    while inside(Str2, fobs, box):
        tellme('Select the Starting position of the 2nd drone')
        Str2 = np.array(plt.ginput(1, timeout=-1))
    ax.scatter(Str2[0][0], Str2[0][1], s=10, facecolors='red')
    
    End2 = []
    while inside(End2, fobs, box):
        tellme('Select the Ending position of the 1st drone')
        End2 = np.array(plt.ginput(1, timeout=-1))
    ax.scatter(End2[0][0], End2[0][1], s=10, facecolors='red')
    
    V02 = np.append(Str2, End2, axis=0)
    return V0, V02
#.......................................................................................................................


def inside(xy, fobs, box):
    ins = False
    dim = 2

    if len(xy) == 0:
        ins = True

    else:
        for i in range(len(fobs)):
            if linalg.norm(fobs[i][0:dim] - xy[0]) < fobs[i][-1]:
                ins = True

        for i in range(box.shape[0]):
            poly = Polygon([(box[i][0]), (box[i][1]), (box[i][2]), (box[i][3])])
            p = Point(xy[0])
            if poly.contains(p):
                ins = True

    return ins

#=======================================================================================================================


def routing_graph(scene, V0, Nv, thres, seed):
    dims = scene.dims
    fobs = scene.fobs
    box = scene.box
    dim = int(len(scene.dims) / 2)

    np.random.seed(seed)                  # Generating random vertices
    nv0 = V0.shape[0]
    V = np.multiply(dims[1::2]-dims[0::2], np.random.rand(Nv, int(dim))) - dims[1::2]
    V = np.concatenate((V, V0))

    removal = []
    for i in range(len(V) - nv0):         # Removing vertices that are too close
        if min(linalg.norm(V[i, :]-V[i+1:-1, :], axis=1)) < thres:
            removal.append(i)
    V = np.delete(V, removal, axis=0)

    removal = []
    for i in range(len(fobs)):            # Removing vertices that are inside a fix-obstacle (circle)
        for j in range(len(V)):
            if linalg.norm(fobs[i][0:dim]-V[j, :], axis=0) < fobs[i][-1]:
                removal.append(j)
    V = np.delete(V, removal, axis=0)

    removal = []
    for i in range(box.shape[0]):         # Removing vertices that are inside a fix-obstacle (polygon)
        poly = Polygon([(box[i][0]), (box[i][1]), (box[i][2]), (box[i][3])])
        for j in range(len(V)):
            p = Point(V[j])
            if poly.contains(p):
                removal.append(j)
    V = np.delete(V, removal, axis=0)

    G = nx.Graph()                        # Adding the vertices to the graph
    for i in range(len(V)):
        G.add_node(i, pos=[V[i]])

    tri = Delaunay(V)                     # Delaunay triangulation of a set of points

    graph_adj = [list() for i in range(len(V))]  # Del_tri:[A,B,C],[A,B,D] -> Adj_graph:A[B,C,D],B[A,C,D],C[A,B],D[A,B]
    for i in range(len(tri.simplices)):
        for j in range(3):
            a = tri.simplices[i, j]
            graph_adj[a].extend(tri.simplices[i])
            graph_adj[a].remove(a)
            graph_adj[a] = remove_duplicates(graph_adj[a])
            if len(fobs) > 0:
                graph_adj[a] = [e for e in graph_adj[a] if not intersect(V[a], V[e], fobs[:, 0:dim], fobs[:, -1], box)]
            else:
                graph_adj[a] = [e for e in graph_adj[a] if not intersect(V[a], V[e], [], [], box)]
                

    for i in range(len(graph_adj)):       # Adding the weighted edges to the graph
        for j in graph_adj[i]:
            G.add_edge(i, j, weight=abs(linalg.norm(V[i]-V[j])))

    return G, V
#.......................................................................................................................


def remove_duplicates(duplist):
    noduplist = []
    for i in duplist:
        if i not in noduplist:
            noduplist.append(i)
    return noduplist
#.......................................................................................................................


def intersect(v1, v2, C, r, box):
    res = False
    line_v = LineString([v1, v2])

    for i in range(len(r)):               # Check if a line between two vertexes intersect a circle
        circ = Point(C[i])
        circ = circ.buffer(r[i]).boundary
        res = res or (circ.intersects(line_v))

    for i in range(box.shape[0]):         # Check if a line between two vertexes intersect a polygon
        line_p = LineString([box[i][0], box[i][1], box[i][2], box[i][3]])
        res = res or (line_v.intersects(line_p))

    return res
#=======================================================================================================================


# def dijkstra_timed(G, V, mobs_obj, drone_v, *, return_idx = None, return_dict = None):
def dijkstra_timed(G, V, mobs_obj, drone_v, return_idx, return_dict):
    cmin = 2                              # cmi and cmax are required for the cost computation
    cmax = 20
    vs = len(V) - 2                       # Starting vertex
    ve = len(V) - 1                       # Ending vertex
    nv = V.shape[0]
    notvisited = np.full(nv, True, dtype=bool)
    prev = np.zeros(nv)
    tt = np.full(nv, np.inf)
    tt[vs] = 0
    cost = np.full(nv, np.inf)
    cost[vs] = 0
    nm = len(mobs_obj)
    coll = np.full(nm, False, dtype=bool)
    collmat = np.full((nv, nm), False, dtype=bool)
    pq = PriorityQueue()
    pq.put((0, vs))

    while not pq.empty():
        (dist, current_vertex) = pq.get()
        if current_vertex == ve:
            break
        notvisited[current_vertex] = False
        t = tt[current_vertex]

        for neighbour in list(G[current_vertex].keys()): # list(G[].keys()) contains the adjacent vertexes
            if notvisited[neighbour]:
                dt = list(G[neighbour][current_vertex].values())[0] / drone_v # list(G[][].val..) contains the distances
                tspan = np.array([t, t + dt])
                cc = 0
                coll = [False and elem for elem in coll]

                for j in range(nm):
                    d = minreldist(V[current_vertex], V[neighbour], j, tspan, mobs_obj)
                    if d:
                        coll[j] = True
                        cc = cc + (cmin+(1-d)*(cmax-cmin)) # if d is not an empty list it will be handled like an int
                        # print("Collision with", j, "Between:", current_vertex, i, "Collision cost:", cc,
                        #       "\nColl:", coll)

                old_cost = cost[neighbour]
                new_cost = cost[current_vertex] + dt + cc * 10
                if new_cost < old_cost:
                    pq.put((new_cost, neighbour))
                    cost[neighbour] = new_cost
                    prev[neighbour] = current_vertex
                    tt[neighbour] = tt[current_vertex] + dt
                    collmat[neighbour, :] = coll

    vk = ve
    k = nv - 1
    route = np.zeros(nv)
    while vk != 0:              # Route building
        route[k] = vk
        k = k - 1
        vk = int(prev[vk])
    route = route.astype(int)
    route = route[k + 1:]        # Delete the zeros
    ttime = tt[route]
    collmat = collmat[route, :]
    #print("Route:\n", route,
    #      "\nTime:\n", ttime,
    #      "\nCollmat\n:", collmat)

    if len(route) == 1:
        sys.exit("Path could not be created. \nThe two selected points should be in a same space.")
        
        
    # if return_dict != None:
    #     return_dict[str(return_idx)] = [route, ttime, collmat]
        
    # print("kkkkkkkkkkkkkkkkkĢkkkkkkkkkkkkkkkkkĢkkkkkkkkkkkkkkkkkĢ")
    return_dict[str(return_idx)] = [route, ttime, collmat]
    return route, ttime, collmat
#.......................................................................................................................


def minreldist(v1, v2, j, tspan, mobs_obj):
    tgrid = np.transpose(np.linspace(tspan[0], tspan[1], 50))
    tgrid = np.reshape(tgrid, tgrid.size)
    C = interpolate.splev(mobs_obj[j].move(tgrid), mobs_obj[j].path)
    line = LineString([v1, v2])
    d = np.zeros(len(C[0]))

    for i in range(len(C[0])):
        p = Point(C[0][i], C[1][i])
        d[i] = p.distance(line)
    if any(k <= mobs_obj[j].r for k in d):
        co = min(d/mobs_obj[j].r)
    else:
        co = []

    return co
#=======================================================================================================================


def flight_sim(mobs_obj, drone_obj, Ts):             # 2. input: animation frame delay in milliseconds
    tellme('Simulation')                                          # TODO: 3. input: time of the animation
    fig = plt.gcf()                       # Catch the current fig which previously created by create_view
    ax = fig.gca()
    patch = []

    for i in range(0, len(mobs_obj)-1):
        patch.append(plt.Circle((0, 0), mobs_obj[i].r,
                                lw=1, edgecolor='blue', facecolor='green', alpha=0.6))
    #patch.append(plt.Circle((0, 0), drone_obj[0].r,
    #                        lw=1, edgecolor='blue', facecolor='red', alpha=0.5))
    patch.append(plt.Circle((0, 0), drone_obj[0].r,
                            lw=2, edgecolor='green', facecolor='black', alpha=1))
    patch.append(plt.Circle((0, 0), drone_obj[0].r,
                            lw=2, edgecolor='blue', facecolor='black', alpha=1))


    def init():
        #patch[-3].center = interpolate.splev(drone_obj[0].move(0), drone_obj[0].path)
        #ax.add_patch(patch[-3])
        patch[-1].center = interpolate.splev(drone_obj[0].gurobi_move(0), drone_obj[0].path)
        ax.add_patch(patch[-1])
        patch[-2].center = interpolate.splev(mobs_obj[-1].move(0), mobs_obj[-1].path)
        ax.add_patch(patch[-2])
        for i in range(0, len(mobs_obj)-1):
            patch[i].center = interpolate.splev(mobs_obj[i].move(0), mobs_obj[i].path)
            ax.add_patch(patch[i])
        return patch

    def update(frame_number):
        #x, y = interpolate.splev(drone_obj[0].move(frame_number * Ts * 0.001), drone_obj[0].path)
        #patch[-3].center = (x, y)
        x, y = interpolate.splev(drone_obj[0].gurobi_move(frame_number * Ts * 0.001), drone_obj[0].path)
        patch[-1].center = (x, y)
        x, y = interpolate.splev(mobs_obj[-1].move(frame_number * Ts * 0.001), mobs_obj[-1].path)
        patch[-2].center = (x, y)
        for i in range(0, len(mobs_obj)-1):
            x, y = interpolate.splev(mobs_obj[i].move(frame_number * Ts * 0.001), mobs_obj[i].path)
            patch[i].center = (x, y)
        return patch

    return FuncAnimation(fig, update, init_func=init, interval=Ts, blit=True)
#=======================================================================================================================


def gurobi_planner(drone_obj, mobs_obj):
    Ts = 0.1
    Buff_list = [1, 2, 3, 4, 5]
    for Buff in Buff_list:
        H = math.ceil(((drone_obj[0].len/drone_obj[0].v)/Ts)*Buff)
        tgrid = np.arange(0, H*Ts, Ts)
        sgrid = np.linspace(0, drone_obj[0].len, 500)
        drone_pos = interpolate.splev(sgrid, drone_obj[0].path)
        table = np.zeros((len(tgrid), 101))
        table[:, 0] = np.transpose(tgrid)
        cntr = 0

        timer_start = time.time()
        for i in range(0, len(mobs_obj)):
            active = False
            mob_pos = interpolate.splev(mobs_obj[i].move(tgrid), mobs_obj[i].path)
            for j in range(0, len(tgrid)):
                svals = []
                Mob_Circle = Point(mob_pos[0][j], mob_pos[1][j]).buffer(mobs_obj[i].r).boundary
                for k in range(len(drone_pos[0])):
                    Drone_Circle = Point(drone_pos[0][k], drone_pos[1][k]).buffer(drone_obj[0].r*1.5).boundary
                    if Drone_Circle.intersects(Mob_Circle):
                        svals.append(sgrid[k])
                if svals:
                    if not active:
                        active = True
                        cntr = cntr + 1
                    table[j, 2*cntr-1], table[j, 2*cntr] = np.amin(svals), np.amax(svals)
                else:
                    active = False
        timer_end = time.time()
        print("Time of table generation for gurobi:", timer_end - timer_start, "sec\n")

        opt_mod = Model(name="linear program")

        a = opt_mod.addVars(H, name='a', vtype=GRB.CONTINUOUS, lb=-drone_obj[0].a_max, ub=drone_obj[0].a_max)
        v = opt_mod.addVars(H+1, name='v', vtype=GRB.CONTINUOUS, lb=0, ub=drone_obj[0].v_max)
        s = opt_mod.addVars(H+1, name='s', vtype=GRB.CONTINUOUS)
        adir = opt_mod.addVars(cntr, name='adir', vtype=GRB.BINARY)

        sB = drone_obj[0].len+1

        opt_mod.addConstr(s[0] == 0)
        opt_mod.addConstr(s[len(s) - 1] == drone_obj[0].len)
        opt_mod.addConstr(v[0] == 0)
        opt_mod.addConstr(v[len(v)-1] == 0)
        opt_mod.addConstrs(v[k + 1] == v[k] + a[k] * Ts
                           for k in range(H))
        opt_mod.addConstrs(s[k + 1] == s[k] + v[k] * Ts + 0.5 * (Ts ** 2) * a[k]
                           for k in range(H))
        opt_mod.addConstrs(s[k] - table[k, 2 * obs + 1] <= adir[obs] * sB
                           for k in range(H)
                           for obs in range(cntr)
                           if table[k, 2 * obs + 1] > 0)
        opt_mod.addConstrs(table[k, 2 * obs + 2] - s[k] <= (1 - adir[obs]) * sB
                           for k in range(H)
                           for obs in range(cntr)
                           if table[k, 2 * obs + 2] > 0)

        #!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        fH = []
        for x in range(len(v)):
            if x <= len(v)/Buff:
                fH.append(0)
            else:
                fH.append(1)
        d = [(v[x] - drone_obj[0].v)*(1-fH[x]) + (drone_obj[0].len - s[x])*(fH[x])*100 for x in range(len(v))]
        d = np.array(d)
        d = d.dot(np.transpose(d)) / H
        opt_mod.setObjective(d)
        # !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

        opt_mod.ModelSense = GRB.MINIMIZE
        opt_mod.optimize()

        try:
            ans = opt_mod.objVal
            print('Buff =', Buff)
            break
        except AttributeError as e:
            pass

    s_opt = [var.x for var in opt_mod.getVars() if "s" in var.VarName]
    s_opt = np.array(s_opt)
    if tgrid.shape != s_opt.shape:
        tgrid = np.append(tgrid, tgrid[-1]+Ts)
    spl = interpolate.splrep(tgrid, s_opt, k=2, s=0)
    drone_obj[0].gp = spl
    drone_obj[0].gt = tgrid[-1]

    results = Results()
    results.s = s_opt
    results.v = [var.x for var in opt_mod.getVars() if "v" in var.VarName]
    results.a = [var.x for var in opt_mod.getVars() if "a" in var.VarName]
    results.tgrid = tgrid

    return results

