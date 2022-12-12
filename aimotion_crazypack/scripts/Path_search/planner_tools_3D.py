import random
import time
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider, Button
from matplotlib import cm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import networkx as nx
from queue import PriorityQueue
from scipy import interpolate
from numpy import linalg
from scipy.spatial import Delaunay
from gurobipy import *
from AllGeo import GeoPoint, GeoPolygon, GeoPolygonProc
import copy
import csv
import pickle


# Not tested -->
def pickle_delete(filename):
    if os.path.exists(filename + ".pickle"):
        os.remove(filename + ".pickle")

def pickle_write(filename, data):
    pickle_out = open(filename + ".pickle", "wb")     # Save scene
    pickle.dump(data, pickle_out)
    pickle_out.close()

def pickle_read(filename):
    if os.path.exists(filename + ".pickle"):
        pickle_in = open(filename + ".pickle", "rb")  # Loading the layout
        res = pickle.load(pickle_in)
        pickle_in.close()
    else:
        res = 0
    return res

# <-- not tested

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
    filename = "dijkstra3D/urdf/" + "pole" + str(ID) + ".urdf"
    f = open(filename, "w")
    f.write(urdf_text)
    f.close()

    return 0


def write_initial_position_yaml(x0_list):
    # Writing initial position
    import yaml
    yaml_dict = {'crazyflies' : []}
    for i in range(len(x0_list)):
        initialPosition = x0_list[i]
        # targetHeight = 0.8
        initialPosition = initialPosition #+ [float(targetHeight)]
        yaml_dict['crazyflies'] += [{'id' : i, 'channel' : 100,
                                     'initialPosition' : initialPosition,
                                     'type' : 'default'
                                     }]

    with open("dijkstra3D/yaml/initialPosition.yaml", "w") as file_descriptor:
                    yaml.dump(yaml_dict, file_descriptor)



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
    filename = "dijkstra3D/urdf/" + "pole" + str(ID) + ".urdf"
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
    n_divisions = 10
    t_divisions = np.linspace(t_start, t_end, n_divisions + 1)
    poly7_x_slices, poly7_y_slices, poly7_z_slices, t_slices = [], [], [], []
    for i in range(1, len(t_divisions)):
        t_slice_start = t_divisions[i - 1]
        t_slice_end = t_divisions[i]
        t_slice = np.linspace(t_slice_start, t_slice_end)
        t_slices.append(list(copy.deepcopy(t_slice)))

        # x, y = interpolate.splev(drone_obj[0].gurobi_move(frame_number * Ts * 0.001), drone_obj[0].path)
        # patch[-2].center = (x, y)
        # for i in range(0, len(mobs_obj)):
        # x, y = interpolate.splev(mobs_obj[i].move(frame_number * Ts * 0.001), mobs_obj[i].path)

        if getattr(type(object_instance[object_id]), '__name__', '') == 'Mobs':
            # x_slice, y_slice = interpolate.splev(object_instance[object_id].move(t_slice), object_instance[object_id].path)
            x_slice, y_slice, z_slice = [], [], []
            try:
                for t_slice_ in t_slice:
                    tmp_x, tmp_y, tmp_z = interpolate.splev(object_instance[object_id].move(t_slice_), object_instance[object_id].path)
                    x_slice.append(tmp_x)
                    y_slice.append(tmp_y)
                    z_slice.append(tmp_z)
            except:
                pass
        elif getattr(type(object_instance[object_id]), '__name__', '') == 'Drone':
            x_slice, y_slice, z_slice = [], [], []
            for t_slice_ in t_slice:
                tmp_x, tmp_y, tmp_z = interpolate.splev(object_instance[object_id].gurobi_move(t_slice_), object_instance[object_id].path)
                x_slice.append(tmp_x)
                y_slice.append(tmp_y)
                z_slice.append(tmp_z)
        else:
            raise NotImplementedError()

        # x_slice, y_slice = interpolate.splev(t_slice, object_instance[object_id].path)

        # Let us correct for the fact, that the crazyflies will evaulate each
        # polynomial piece between t \in [0, t_desired]
        t_shift = t_slices[i-1][0]
        t_slices[i-1] -= t_shift

        deg = 7
        poly7_x_slice = np.poly1d(np.polyfit(t_slices[i-1], x_slice, deg=deg))
        poly7_y_slice = np.poly1d(np.polyfit(t_slices[i-1], y_slice, deg=deg))
        poly7_z_slice = np.poly1d(np.polyfit(t_slices[i-1], z_slice, deg=deg))

        # Sometimes, for example when all the points are exactly zero, polyfit simply returns a single coefficient, 0
        # Therefore, if the number of coefficients is not deg+1, then let us add deg zeros to the beginning.
        # TODO: currently we assume, that the higher degrees are zeroed-out. But we should check this more carefully...
        if len(poly7_x_slice.coeffs) != deg+1:
            added_length = deg + 1 - len(poly7_x_slice.coeffs)
            poly7_x_slice = np.poly1d(np.append(np.ones(added_length)*1e-10, poly7_x_slice.coeffs))
            assert len(poly7_x_slice.coeffs) == deg+1
        if len(poly7_y_slice.coeffs) != deg+1:
            added_length = deg + 1 - len(poly7_y_slice.coeffs)
            poly7_y_slice = np.poly1d(np.append(np.ones(added_length)*1e-10, poly7_y_slice.coeffs))
            assert len(poly7_y_slice.coeffs) == deg+1
        if len(poly7_z_slice.coeffs) != deg+1:
            added_length = deg + 1 - len(poly7_z_slice.coeffs)
            poly7_z_slice = np.poly1d(np.append(np.ones(added_length)*1e-10, poly7_z_slice.coeffs))
            assert len(poly7_z_slice.coeffs) == deg+1

        poly7_x_slices.append(poly7_x_slice)
        poly7_y_slices.append(poly7_y_slice)
        poly7_z_slices.append(poly7_z_slice)


    # Let's check, if we get something useful back
    for i in range(len(t_slices)):
        x = poly7_x_slices[i](t_slices[i])
        y = poly7_y_slices[i](t_slices[i])
        z = poly7_z_slices[i](t_slices[i])
        # plt.plot(x, y, 'k')


    # Let us slow it down 5 fold
    slow_down = 1
    for slice_count in range(len(poly7_x_slices)):
        power = 0
        for i in range(len(poly7_x_slices[slice_count].coeffs)-1, 0-1, -1):
            poly7_x_slices[slice_count].coeffs[i] = poly7_x_slices[slice_count].coeffs[i] / pow(slow_down, power)
            try:
                poly7_y_slices[slice_count].coeffs[i] = poly7_y_slices[slice_count].coeffs[i] / pow(slow_down, power)
            except:
                print(i)
                print(slice_count)
                print(len(poly7_y_slices[slice_count].coeffs))
                assert len(poly7_y_slices[slice_count].coeffs) != 8
            poly7_z_slices[slice_count].coeffs[i] = poly7_z_slices[slice_count].coeffs[i] / pow(slow_down, power)
            power += 1


    # Let us write the whole thing into a csv file.

    # First, we need to revert the order of the coefficients, because the
    # Crazyflie wants it the other way around
    poly7_x_coeffs, poly7_y_coeffs, poly7_z_coeffs, t_desired = [], [], [], []
    for i in range(len(t_slices)):
        poly7_x_coeffs.append(poly7_x_slices[i].coeffs.tolist()[::-1])
        poly7_y_coeffs.append(poly7_y_slices[i].coeffs.tolist()[::-1])
        poly7_z_coeffs.append(poly7_z_slices[i].coeffs.tolist()[::-1])
        t_desired.append(t_slices[i][-1] * slow_down)

    # We should hover at the end
    # if getattr(type(object_instance[object_id]), '__name__', '') == 'Drone':
    if object_name == 'vehicle':
        if getattr(type(object_instance[object_id]), '__name__', '') == 'Drone':
            hower_x, hower_y, hower_z = interpolate.splev(object_instance[object_id].gurobi_move(object_instance[object_id].gt), object_instance[object_id].path)
        else:
            hower_x, hower_y, hower_z = interpolate.splev(object_instance[object_id].move(object_instance[object_id].gt), object_instance[object_id].path)
        hower_zeros = [0] * 7
        poly7_x_coeffs.append([hower_x.tolist()] + hower_zeros)
        poly7_y_coeffs.append([hower_y.tolist()] + hower_zeros)
        poly7_z_coeffs.append([hower_z.tolist()] + hower_zeros)
        t_desired.append(0.01)




    if file_idx == None:
        write_csv(t_desired, poly7_x_coeffs, poly7_y_coeffs, poly7_z_coeffs, object_name + str(object_id))
    else:
        write_csv(t_desired, poly7_x_coeffs, poly7_y_coeffs, poly7_z_coeffs, object_name + str(file_idx))


    return 0


def write_csv(T, px, py, pz, name):

    # px, py, pz, pj = [0], [0], [0], [0]

    pj = [0] * len(px[0])

    # Writing this first line is required by crazyswarm
    first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']
    mode = 'w'
    with open('dijkstra3D/csv/' + name + '.csv', mode = mode) as csvfile:
        writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        writer.writerow(first_line)
        for i in range(len(T)):
            writer.writerow([T[i]] + px[i] + py[i] + pz[i] + pj)
    return 0


# -----------------------------------------------------------------------------------
class Results:  # We use it to store base data.
    def __init__(self):
        self.s = []
        self.v = []
        self.a = []
        self.tgrid = []


# ----------------------------------------------------------------------------------------------
xyz_path = []
xyz_list = []


# ----------------------------------------------------------------------------------------------------------------------
class Scene:  # We use it to store base data.
    def __init__(self):
        self.dims = []
        self.fobs = []
        self.box = []
        self.L_zone_num = []
        self.boxminmax = []
        self.mobs = []
        self.G = []
        self.V = []
        self.V0 = []


# =======================================================================================================================
class Mobs:  # We use it to store the data and functions required for the animation.
    def __init__(self, radius, velocity, path, path_length, mode, tgrid, C, gp, gt):
        self.r = radius
        self.v = velocity
        self.path = path
        self.len = path_length
        self.mode = mode
        self.tgrid = tgrid
        self.C = C
        self.gp = gp
        self.gt = gt

    def move(self, t):  # Define the types of movements
        if self.mode == "res":  # If the end of the path is reached restart from the start
            s = np.remainder(self.v * t, self.len)
            return s
        elif self.mode == "bnf":  # If the end/start of the path is reached switch direction
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


# .......................................................................................................................
def back_n_forth(t, act_len):  # Helping function to create the back and forth motion
    d = np.floor_divide(t, act_len)
    r = t - d * act_len
    odd = np.remainder(d, 2)
    tt = np.multiply((1 - odd), r) + np.multiply(odd, (act_len - r))
    return tt


# =======================================================================================================================
class Dummies:  # We use it to store the data and functions required for the animation.
    def __init__(self, radius, height, velocity, path, path_length, tgrid, C, gp, gt):
        self.r = radius
        self.v = velocity
        self.h = height
        self.path = path
        self.len = path_length
        self.tgrid = tgrid
        self.C = C
        self.gp = gp
        self.gt = gt

    def move(self, t):  # Define the types of movements
        t_max = self.len/self.v
        #try:
        #s = np.remainder(self.v * min(t, t_max-0.001), self.len) # For movement
        #except:
        #    s = np.remainder(self.v * t, self.len)  # For tgrid
        s = []
        try:
            s = np.remainder(self.v * t, self.len)
        except ValueError:
            for t_ in t:
                s.append(np.remainder(self.v * min(t_, t_max-0.001), self.len).tolist())
            s = np.array(s)
        return s


# ======================================================================================================================
class Wanderer: # Moving obstacle with unknown movement
    def __init__(self, radius, start_position, start_velocity, start_orientations, rand_seed, max_difference, cage):
        self.r = radius
        self.p = start_position
        self.v = start_velocity
        self.alpha = start_orientations[0]
        self.beta = start_orientations[1]
        self.seed = rand_seed
        self.max = max_difference
        self.cage = cage

    def move(self, t):
        np.random.seed(self.seed + int(t))
        self.v = max(-0.05, min(0.05, self.v + (random.randint(self.max[0]*10000, self.max[0]*10000)/10000)))
        np.random.seed(self.seed + int(t) + 1)
        self.alpha = self.alpha + (random.randint(-self.max[1], self.max[1]))
        np.random.seed(self.seed + int(t) + 2)
        self.beta = self.beta + (random.randint(-self.max[1], self.max[1]))
        self.p[0] = max(min(self.p[0] + self.v * math.cos(math.radians(self.beta)) * math.sin(math.radians(self.alpha)),
                    self.cage[1]), self.cage[0])
        self.p[1] = max(min(self.p[1] + self.v * math.cos(math.radians(self.beta)) * math.cos(math.radians(self.alpha)),
                    self.cage[3]), self.cage[2])
        self.p[2] = max(min(self.p[2] + self.v * math.sin(math.radians(self.beta)), self.cage[5]), self.cage[4])
        return [self.p[0], self.p[1], self.p[2]]

# ======================================================================================================================
class Drone:  # We use it to store the data and functions required for the animation.
    def __init__(self, radius, velocity, path, path_length, gurobi_path, gurobi_time):
        self.r = radius
        self.v = velocity
        self.v_max = 0.8
        self.a_max = 0.4
        self.path = path
        self.len = path_length
        self.gp = gurobi_path
        self.gt = gurobi_time

    def move(self, t):  # Define the types of movements
        s = np.remainder(self.v * min(t, self.gt), self.len)
        return s

    def gurobi_move(self, t):
        try:
            sg = interpolate.splev(min(t, self.gt), self.gp)
            return sg

        except ValueError:
            s = []
            for t_ in t:
                s.append(interpolate.splev(min(t_, self.gt), self.gp).tolist())
            return np.array(s)


# ======================================================================================================================
def create_view(scene, vis_mobs):  # Create the scene and show the mob paths if required
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlim3d(scene.dims[0], scene.dims[1])
    ax.set_ylim3d(scene.dims[2], scene.dims[3])
    #ax.set_zlim3d(scene.dims[4], scene.dims[5])
    #ax.set_xlim3d(-3.5, 3.5)
    #ax.set_ylim3d(-3.5, 3.5)
    ax.set_zlim3d(0, 3)
    plt.gca().set_aspect('auto', adjustable='box')
    plt.grid(True)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    if len(scene.fobs):  # Plotting the fix-obstacles if there is any
        C = scene.fobs[:, 0:3]  # Separate the koordinates of the fix-obstacles
        r = scene.fobs[:, -1]  # Separate the radii of the fix-obstacles
        for i in range(0, C.shape[0]):  # Draw the fix-obstacles
            u = np.linspace(0, 2 * np.pi, 20)
            v = np.linspace(0, np.pi, 20)
            x = r[i] * np.outer(np.cos(u), np.sin(v))
            y = r[i] * np.outer(np.sin(u), np.sin(v))
            z = r[i] * np.outer(np.ones(np.size(u)), np.cos(v))
            ax.plot_surface(x + C[i][0], y + C[i][1], z + C[i][2], cmap=cm.viridis, alpha=0.5)

    if len(scene.box):  # Plotting the fix-obstacles if there is any
        for i in range(scene.box.shape[0]):
            p1 = scene.box[i][0]+[+0.12,+0.12,+0,] # remove the radius of the drones
            p2 = scene.box[i][1]+[-0.12,+0.12,+0,]
            p3 = scene.box[i][2]+[-0.12,-0.12,+0,]
            p4 = scene.box[i][3]+[+0.12,-0.12,+0,]

            p5 = scene.box[i][4]+[+0.12,-0.12,+0,]
            p6 = scene.box[i][5]+[+0.12,+0.12,+0,]
            p7 = scene.box[i][6]+[-0.12,+0.12,+0,]
            p8 = scene.box[i][7]+[-0.12,-0.12,+0,]

            if p8[2] > 1:
                x = [p1[0], p2[0], p3[0], p4[0]]
                y = [p1[1], p2[1], p3[1], p4[1]]
                z = [p1[2], p2[2], p3[2], p4[2]]
                verts = [list(zip(x, y, z))]
                ax.add_collection3d(Poly3DCollection(verts, facecolors='darkblue', alpha=0.7))
                x = [p5[0], p6[0], p7[0], p8[0]]
                y = [p5[1], p6[1], p7[1], p8[1]]
                z = [p5[2], p6[2], p7[2], p8[2]]
                verts = [list(zip(x, y, z))]
                ax.add_collection3d(Poly3DCollection(verts, facecolors='yellow', alpha=0.7))
                x = [p1[0], p6[0], p5[0], p4[0]]
                y = [p1[1], p6[1], p5[1], p4[1]]
                z = [p1[2], p6[2], p5[2], p4[2]]
                verts = [list(zip(x, y, z))]
                ax.add_collection3d(Poly3DCollection(verts, facecolors='green', alpha=0.7))
                x = [p1[0], p6[0], p7[0], p2[0]]
                y = [p1[1], p6[1], p7[1], p2[1]]
                z = [p1[2], p6[2], p7[2], p2[2]]
                verts = [list(zip(x, y, z))]
                ax.add_collection3d(Poly3DCollection(verts, facecolors='green', alpha=0.7))
                x = [p3[0], p8[0], p5[0], p4[0]]
                y = [p3[1], p8[1], p5[1], p4[1]]
                z = [p3[2], p8[2], p5[2], p4[2]]
                verts = [list(zip(x, y, z))]
                ax.add_collection3d(Poly3DCollection(verts, facecolors='green', alpha=0.7))
                x = [p8[0], p7[0], p2[0], p3[0]]
                y = [p8[1], p7[1], p2[1], p3[1]]
                z = [p8[2], p7[2], p2[2], p3[2]]
                verts = [list(zip(x, y, z))]
                ax.add_collection3d(Poly3DCollection(verts, facecolors='green', alpha=0.7))
    """
    if len(scene.mobs):  # Plotting all existing or required moving-obstacle paths
        j = 0
        for i in scene.mobs:
            if j in vis_mobs or 0 == len(vis_mobs):
                path = Path_creator(i, "norm")
                X_path, Y_path, Z_path = path
                plt.plot(X_path, Y_path, Z_path, 'blue', lw=1.5, alpha=1)
            j = j + 1
    """


# ======================================================================================================================
def Path_creator(xy, mode):
    x = []
    y = []
    z = []
    len_xy = len(xy)  # Preparing data to set the degree of the splines

    if len(xy[0]) == 2:
        if xy[0, 0] == xy[-1, 0] and xy[0, 1] == xy[-1, 1]:
            Closed_path = True  # Check if a path is closed or not
        else:
            Closed_path = False

    for point in xy:  # Separate the x y coordinates
        x.append(point[0])
        y.append(point[1])
        if len(xy[0]) == 3:
            z.append(point[2])

    if len(xy[0]) == 2:
        if Closed_path:  # Find the B-spline representation of the xy points
            tck, *rest = interpolate.splprep([x, y], k=3, s=0)  # Closed path
        elif len_xy == 2:
            tck, *rest = interpolate.splprep([x, y], k=1, s=0)  # Line
        else:
            tck, *rest = interpolate.splprep([x, y], k=2, s=0)  # Curve

    if len(xy[0]) == 3:
        if len(xy) == 2:
            tck, *rest = interpolate.splprep([x, y, z], k=1, s=0)  # Line
        else:
            tck, *rest = interpolate.splprep([x, y, z], k=2, s=0)  # Curve

    u = np.arange(0, 1.01, 0.01)  # Set the breaks of the path
    path = interpolate.splev(u, tck)  # Make the path (x,y point sequence)

    if mode == "norm":  # In normal mode just return this path
        return path
    elif mode == "csnp":
        if len(xy[0]) == 2:
            X_path, Y_path = path  # Separate the x and y coordinates ('dense' in the original matlab code)
        if len(xy[0]) == 3:
            X_path, Y_path, Z_path = path
        D = np.diff(path)  # Calculate the discrete differences between the points
        length = [0, np.cumsum(math.sqrt(np.sum(np.multiply(D, D)))) * 10]  # Get the length of the path
        par = np.transpose(np.linspace(length[0], length[-1], 101))  # Create length accurate breaks
        par = np.reshape(par, par.size)  # convert the 2D array to a 1D array
        if len(xy[0]) == 2:
            tck, u, *rest = interpolate.splprep([X_path, Y_path], k=2, s=0, u=par)
        if len(xy[0]) == 3:
            tck, u, *rest = interpolate.splprep([X_path, Y_path, Z_path], k=2, s=0, u=par)
        return tck, u  # Find the B-spline representation with accurate break lengths
    else:
        print("ERROR: Path_creator.mode (req: 'norm' or 'csnp')")


# ======================================================================================================================
def tellme(s):  # Helping function to write out instructions for the making the paths
    print(s)
    plt.title(s, fontsize=16)
    plt.draw()


# ======================================================================================================================
def get_path(xy_list, V0):  # Take a point sequence by mouse clicks from the scene
    fig = plt.gcf()  # Catch the current fig which previously created by create_view
    ax = fig.gca()
    plt.subplots_adjust(left=0.25)
    slider_ax = plt.axes([0.05, 0.25, 0.0225, 0.63])  # left, right, height, width
    x_slider = Slider(ax=slider_ax, label='X', orientation="vertical",
                      valmin=-5, valmax=5, valinit=0)
    slider_ax = plt.axes([0.12, 0.25, 0.0225, 0.63])  # left, right, height, width
    y_slider = Slider(ax=slider_ax, label='Y', orientation="vertical",
                      valmin=-5, valmax=5, valinit=0)
    slider_ax = plt.axes([0.19, 0.25, 0.0225, 0.63])  # left, right, height, width
    z_slider = Slider(ax=slider_ax, label='Z', orientation="vertical",
                      valmin=-5, valmax=5, valinit=0)
    button_ax = plt.axes([0.05, 0.15, 0.12, 0.04])
    marker_button = Button(button_ax, 'Mark', hovercolor='0.975')
    button_ax = plt.axes([0.05, 0.08, 0.12, 0.04])
    draw_path_button = Button(button_ax, 'Draw path', hovercolor='0.975')

    r = 0.05
    u = np.linspace(0, 2 * np.pi, 10)
    v = np.linspace(0, np.pi, 10)
    x = r * np.outer(np.cos(u), np.sin(v))
    y = r * np.outer(np.sin(u), np.sin(v))
    z = r * np.outer(np.ones(np.size(u)), np.cos(v))
    marker = [ax.plot_surface(x, y, z, color='red')]

    for i in V0:
        ax.plot_surface(x+i[0], y+i[1], z+i[2], color='blue')

    marked = []

    def update(val):
        marker[0].remove()
        marker[0] = ax.plot_surface(x + x_slider.val, y + y_slider.val, z + z_slider.val, color='red')

    def set_marker(val):
        marked.append([ax.plot_surface(x + x_slider.val, y + y_slider.val, z + z_slider.val, color='black')])
        global xyz_path
        xyz_path.append([x_slider.val, y_slider.val, z_slider.val])

    def draw_3dspline(val):
        global xyz_path
        global xyz_list
        if len(xyz_path) < 2:  # Make sure that the user gives at least 2 points
            print('Select min 2 points with left mouse click\n'
                  'Stop adding points with right mouse click')
        else:
            xyz_list.append(xyz_path)
            path = Path_creator(xyz_path, "norm")  # Make a path from the points
            X_path, Y_path, Z_path = path  # Separate the x and y coordinates
            ax.plot(X_path, Y_path, Z_path, 'black', lw=2)  # Draw the path to the scene
            xyz_path = []

    x_slider.on_changed(update)
    y_slider.on_changed(update)
    z_slider.on_changed(update)
    marker_button.on_clicked(set_marker)
    draw_path_button.on_clicked(draw_3dspline)
    plt.show()

    global xyz_list
    return xyz_list


# ======================================================================================================================
def select_SE(V, V0, idx, SE_list,  predef_SE, pre_mode):  # Take a point sequence by mouse clicks from the scene

    if pre_mode:
        vs = predef_SE[idx][0]
        ve = predef_SE[idx][1]
        drones_SE = copy.deepcopy(predef_SE) # Contains the other drones start and end positions
        drones_SE.remove(predef_SE[idx])

        for i in range(len(drones_SE)):
            for j in range(len(drones_SE[i])):
                drones_SE[i][j] = drones_SE[i][j] + len(V) - len(V0)

        vs = len(V) - len(V0) + vs
        ve = len(V) - len(V0) + ve
        return vs, ve, drones_SE

    else:
        if idx == 0:
            print("Available V0 points:")
            for i in range(len(V0)):
                print("Index num:", i, "Coordinates:", V0[i])
            vs_list, ve_list = [-1], [-1]
        else:
            vs_list = [item[0] for item in SE_list]
            ve_list = [item[1] for item in SE_list]
        print("\nDrone number", idx)
        vs = -1
        while 0 > vs or vs > len(V0)-1 or len(V)-len(V0)+vs in vs_list:
            if vs != -1:
                print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                      "\nERROR: This is an other drone's start point OR the index is out of range\n"
                      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            vs = ve = int(input("\nChoose a START point from the list above and enter it's 'Index number': "))
        while vs == ve or 0 > ve or ve > len(V0)-1 or len(V)-len(V0)+ve in ve_list:
            if vs != ve:
                print("\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                      "\nERROR: This is an other drone's end point OR the index is out of range\n "
                      "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
            ve = int(input("\nChoose an END point from the list above and enter it's 'Index number': "))

        vs = len(V)-len(V0)+vs
        ve = len(V)-len(V0)+ve
        return vs, ve

# ......................................................................................................................
def inside(xy, fobs, box):
    ins = False
    dim = len(fobs[0]) - 1

    if len(xy) == 0:
        ins = True

    else:
        for i in range(len(fobs)):
            if linalg.norm(fobs[i][0:dim] - xy[0]) < fobs[i][-1]:
                ins = True

        for i in range(box.shape[0]):
            p = []
            for j in range(8):
                p.append(GeoPoint(box[i][j][0], box[i][j][1], box[i][j][2]))
            gp = [p[0], p[1], p[2], p[3],
                  p[4], p[5], p[6], p[7]]
            gpInst = GeoPolygon(gp)
            procInst = GeoPolygonProc(gpInst)
            if procInst.x0 < xy[0][0] < procInst.x1 and procInst.y0 < xy[0][1] < procInst.y1 and \
                    procInst.z0 < xy[0][2] < procInst.z1:
                if procInst.PointInside3DPolygon(xy[0][0], xy[0][1], xy[0][2]):
                    ins = True

    return ins
# =======================================================================================================================
def get_stick_pos():
    stick_pos = []
    with open('dijkstra3D/csv/Saved_positions.csv', newline='') as csvfile:
        spamreader = csv.reader(csvfile)
        for row in spamreader:
            for i in row:
                try:
                    stick_pos.append(float(i))
                except ValueError:
                    pass

    stick_pos = np.array(stick_pos)
    stick_pos = np.reshape(stick_pos, (-1, 3))

    for i in stick_pos:         # Y and Z swap
        i[1], i[2] = i[2], i[1]

    removal = []
    for i in range(len(stick_pos) - 1):  # Removing vertices that are too close
        if min(linalg.norm(stick_pos[i, :] - stick_pos[i + 1:, :], axis=1)) < 0.1:
            removal.append(i)
    stick_pos = np.delete(stick_pos, removal, axis=0)

    return stick_pos.tolist()

# =======================================================================================================================
def routing_graph(scene, V0, Nv, thres, seed, vis):
    dims = scene.dims
    fobs = scene.fobs
    box = scene.box
    bmm = scene.boxminmax
    dim = int(len(scene.dims) / 2)

    x_points = np.arange(dims[0], dims[1], 0.5)
    y_points = np.arange(dims[2], dims[3], 0.5)
    z_points = np.arange(dims[4], dims[5], 0.5)

    Xv, Yv = np.meshgrid(x_points, y_points)
    Xh, Zh = np.meshgrid(x_points, z_points)
    Yh, Zh = np.meshgrid(y_points, z_points)

    Xv = Xv.reshape((np.prod(Xv.shape),))
    Yv = Yv.reshape((np.prod(Yv.shape),))
    Zh = Zh.reshape((np.prod(Zh.shape),))
    Yh = Yh.reshape((np.prod(Yh.shape),))
    Xh = Xh.reshape((np.prod(Xh.shape),))

    bottom = np.ones(len(Xv))*dims[4]
    cage = list(zip(Xv, Yv, bottom))
    top = np.ones(len(Xv)) * dims[5]
    cage = np.concatenate((cage, list(zip(Xv, Yv, top))))
    front = np.ones(len(Zh)) * dims[0]
    cage = np.concatenate((cage, list(zip(front, Yh, Zh))))
    back = np.ones(len(Zh)) * dims[1]
    cage = np.concatenate((cage, list(zip(back, Yh, Zh))))
    left = np.ones(len(Zh)) * dims[2]
    cage = np.concatenate((cage, list(zip(Xh, left, Zh))))
    right = np.ones(len(Zh)) * dims[3]
    cage = np.concatenate((cage, list(zip(Xh, right, Zh))))

    np.random.seed(seed)  # Generating random vertices
    nv0 = V0.shape[0]
    V = np.multiply(dims[1::2] - dims[0::2], np.random.rand(Nv, int(dim))) - [dims[1], dims[3], -dims[4]]
    V = np.concatenate((V, cage))
    V = np.concatenate((V, V0))

    removal = []
    for i in range(len(V) - nv0 - 1):  # Removing vertices that are too close
        if min(linalg.norm(V[i, :] - V[i + 1:, :], axis=1)) < thres:
            removal.append(i)
    V = np.delete(V, removal, axis=0)

    removal = []
    for i in range(len(V)):
        a = [V[i]]
        if inside(a, fobs, box):
            removal.append(i)
    V = np.delete(V, removal, axis=0)

    if vis:
        fig = plt.gcf()  # Catch the current fig which previously created by create_view
        ax = fig.gca()
        for i in range(len(V) - 1):
           ax.scatter(V[i][0], V[i][1], V[i][2], s=1, alpha=0.5, c='black')

    G = nx.Graph()  # Adding the vertices to the graph
    for i in range(len(V)):
        G.add_node(i, pos=[V[i]])

    tri = Delaunay(V)  # Delaunay triangulation of a set of points
    graph_adj = [list() for i in range(len(V))]  # Del_tri:[A,B,C],[A,B,D] -> Adj_graph:A[B,C,D],B[A,C,D],C[A,B],D[A,B]
    for i in range(len(tri.simplices)):
        for j in range(dim + 1):
            a = tri.simplices[i, j]
            graph_adj[a].extend(tri.simplices[i])
            graph_adj[a].remove(a)
            graph_adj[a] = remove_duplicates(graph_adj[a])
            graph_adj[a] = [e for e in graph_adj[a] if not intersect(V[a], V[e], fobs[:, 0:dim], fobs[:, -1], box, bmm)]

    for i in range(len(graph_adj)):  # Adding the weighted edges to the graph
        for j in graph_adj[i]:
            G.add_edge(i, j, weight=abs(linalg.norm(V[i] - V[j])))

    pos = nx.get_node_attributes(G, 'pos')
    for i, j in enumerate(G.edges()):
        x = np.array((pos[j[0]][0][0], pos[j[1]][0][0]))
        y = np.array((pos[j[0]][0][1], pos[j[1]][0][1]))
        z = np.array((pos[j[0]][0][2], pos[j[1]][0][2]))

        if vis:
            ax.plot(x, y, z, c='black', lw=0.2, alpha=0.5)

    return G, V


# ......................................................................................................................
def remove_duplicates(duplist):
    noduplist = []
    for i in duplist:
        if i not in noduplist:
            noduplist.append(i)
    return noduplist


# ......................................................................................................................
def intersect(v1, v2, C, r, box, bmm):
    res = False

    # Spheres
    if v1[0] - v2[0] == 0 and v1[1] - v2[1] == 0 and len(v1) == 2: # v1[2] == v2[2]
        dist = linalg.norm(C - v1, axis=1, ord=2)
    else:
        alpha = ((C - v1).dot(np.transpose(v2 - v1))) / ((linalg.norm(v2 - v1)) ** 2)
        alpha = np.array([max(min(x, 1), 0) for x in alpha])

        Cp = []
        for i in range(len(v1)):
            Cp.append(v1[i] + (v2[i] - v1[i]) * alpha)
        Cp = np.transpose(np.array(Cp))
        dist = linalg.norm(C - Cp, axis=1, ord=2)
    res = res or (dist < r).any()

    if box != []:
        # Boxes
        xmin = min(v1[0], v2[0])
        xmax = max(v1[0], v2[0])
        ymin = min(v1[1], v2[1])
        ymax = max(v1[1], v2[1])
        zmin = min(v1[2], v2[2])
        zmax = max(v1[2], v2[2])
        for i in range(box.shape[0]):
            if xmax > bmm[i].x0 and xmin < bmm[i].x1 and\
                    ymax > bmm[i].y0 and ymin < bmm[i].y1 and\
                    zmax > bmm[i].z0 and zmin < bmm[i].z1:

                a1, b1 = equation_plane(box[i][0], box[i][1], box[i][2])
                a2, b2 = equation_plane(box[i][4], box[i][6], box[i][5])
                a3, b3 = equation_plane(box[i][0], box[i][4], box[i][5])
                a4, b4 = equation_plane(box[i][0], box[i][5], box[i][6])
                a5, b5 = equation_plane(box[i][2], box[i][7], box[i][4])
                a6, b6 = equation_plane(box[i][7], box[i][1], box[i][6])

                dist = multiDimenDist(v1, v2)
                q = findVec(v1, v2, True)

                opt_mod = Model("intersection")
                opt_mod.setParam('OutputFlag', False)
                opt_mod.setParam('TimeLimit', 0.01)
                lmd = opt_mod.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=dist, name='lmd')
                opt_mod.setObjective(1, GRB.MINIMIZE)
                opt_mod.addConstr(a1.dot(np.transpose(v1)) + lmd * (a1.dot(np.transpose(q))) <= b1)
                opt_mod.addConstr(a2.dot(np.transpose(v1)) + lmd * (a2.dot(np.transpose(q))) <= b2)
                opt_mod.addConstr(a3.dot(np.transpose(v1)) + lmd * (a3.dot(np.transpose(q))) <= b3)
                opt_mod.addConstr(a4.dot(np.transpose(v1)) + lmd * (a4.dot(np.transpose(q))) <= b4)
                opt_mod.addConstr(a5.dot(np.transpose(v1)) + lmd * (a5.dot(np.transpose(q))) <= b5)
                opt_mod.addConstr(a6.dot(np.transpose(v1)) + lmd * (a6.dot(np.transpose(q))) <= b6)
                opt_mod.optimize()

                try:
                    lmd = opt_mod.objVal  # Inside
                    res = res or True
                except AttributeError as e:
                    pass

    return res


# ......................................................................................................................
def equation_plane(p1, p2, p3):
    # These two vectors are in the plane
    v1 = p3 - p1
    v2 = p2 - p1
    # the cross product is a vector normal to the plane
    cp = np.cross(v1, v2)
    a = cp
    # This evaluates a * x3 + b * y3 + c * z3 which equals d
    b = np.dot(cp, p3)
    return a, b


def multiDimenDist(point1, point2):
    #find the difference between the two points, its really the same as below
    deltaVals = [point2[dimension]-point1[dimension] for dimension in range(len(point1))]
    runningSquared = 0
    #because the pythagarom theorm works for any dimension we can just use that
    for coOrd in deltaVals:
        runningSquared += coOrd**2
    return runningSquared**(1/2)


def findVec(point1, point2, unitSphere):
    #unitSphere to True will make the vector scaled down to a sphere with a radius one, instead of it's orginal length
    finalVector = [0 for coOrd in point1]
    for dimension, coOrd in enumerate(point1):
        #finding total differnce for that co-ordinate(x,y,z...)
        deltaCoOrd = point2[dimension]-coOrd
        #adding total difference
        finalVector[dimension] = deltaCoOrd
    if unitSphere:
        totalDist = multiDimenDist(point1, point2)
        unitVector = []
        for dimen in finalVector:
            unitVector.append(dimen/totalDist)
        return unitVector
    else:
        return finalVector


# ======================================================================================================================
def dijkstra_timed(inp_):
    Gb = inp_[0]
    Vb = inp_[1]
    mobs_obj = inp_[2]
    dummies_obj = inp_[3]
    drone_v = inp_[4]
    vs = inp_[5]
    ve = inp_[6]
    emergency_rerouting = inp_[7]
    drone_r = inp_[8]
    drones_SE = inp_[9]

    G = copy.deepcopy(Gb)
    V = copy.deepcopy(Vb[:, :2]) # 2D

    for i in range(len(drones_SE)):
        end_position = Vb[drones_SE[i]][1]
        start_position = Vb[drones_SE[i]][0]
        se_pos = np.append(start_position, end_position)
        se_pos = se_pos.reshape(-1, 3)
        removal = []
        for j in range(len(Vb)):  # Removing vertices that are too close
            de = linalg.norm(Vb[j] - end_position, axis=0)
            ds = linalg.norm(Vb[j] - start_position, axis=0)
            if de < 2 * drone_r or ds < 2 * drone_r:
                #ax.scatter(Vb[j][0], Vb[j][1], Vb[j][2], c='red')
                V[j] = None
                removal.append(j)
            else:                  # if the node itself does not get deleted with all of it's edgeds, check the edges
                try:
                    for neighbour in list(G[j].keys()):
                        V_max = np.maximum(Vb[j],Vb[neighbour])
                        V_min = np.minimum(Vb[j], Vb[neighbour])
                        if (V_max[0] > start_position[0]-2*drone_r and V_min[0] < start_position[0]+2*drone_r and
                            V_max[1] > start_position[1]-2*drone_r and V_min[1] < start_position[1]+2*drone_r and
                            V_max[1] > start_position[1]-2*drone_r and V_min[1] < start_position[1]+2*drone_r) or\
                           (V_max[0] > end_position[0]-2*drone_r and V_min[0] < end_position[0]+2*drone_r and
                            V_max[1] > end_position[1]-2*drone_r and V_min[1] < end_position[1]+2*drone_r and
                            V_max[1] > end_position[1]-2*drone_r and V_min[1] < end_position[1]+2*drone_r):
                            if intersect(Vb[j], Vb[neighbour], se_pos, np.array([2 * drone_r, 2 * drone_r]), [], []):
                                G.remove_edge(j, neighbour)
                                #plt.plot([Vb[j][0], Vb[neighbour][0]], [Vb[j][1], Vb[neighbour][1]],
                                 #        [Vb[j][2], Vb[neighbour][2]], 'red', lw=1.5, alpha=0.5)
                            #plt.plot([Vb[j][0],Vb[neighbour][0]], [Vb[j][1],Vb[neighbour][1]],
                             #[Vb[j][2],Vb[neighbour][2]], 'purple', lw=0.5, alpha=0.5)
                except KeyError:
                    pass
        G.remove_nodes_from(removal)
    #plt.show()
    if emergency_rerouting:
        #fig = plt.gcf()
        #ax = fig.gca()
        V = copy.deepcopy(Vb) # 3D
        for j in range(len(dummies_obj)):
            end_positions = np.array(interpolate.splev(dummies_obj[j].move(10000), dummies_obj[j].path)).reshape(-1, )
            removal = []
            for i in range(len(V)):  # Removing vertices that are too close
                d = linalg.norm(V[i, :2] - end_positions[:2], axis=0)
                if d < 1.5*(drone_r+dummies_obj[j].r) and i != vs:
                    V[i] = None
                    removal.append(i)
                    #ax.scatter(V[i][0], V[i][1], V[i][2])
                else:
                    try:
                        for neighbour in list(G[i].keys()):
                            if intersect(V[i, :2], V[neighbour, :2], np.array([end_positions[:2]]), np.array([dummies_obj[j].r + drone_r]), [], []):
                                G.remove_edge(i, neighbour)
                    except KeyError:
                        pass
            G.remove_nodes_from(removal)
            print("Removed nodes:", removal)

    cmin = 2  # cmi and cmax are required for the cost computation
    cmax = 20
    nv = V.shape[0]
    notvisited = np.full(nv, True, dtype=bool)
    prev = np.zeros(nv)
    tt = np.full(nv, np.inf)
    tt[vs] = 0
    cost = np.full(nv, np.inf)
    cost[vs] = 0
    nm = len(mobs_obj) + len(dummies_obj)
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

        for neighbour in list(G[current_vertex].keys()):  # list(G[].keys()) contains the adjacent vertexes
            if notvisited[neighbour]:
                dt = list(G[neighbour][current_vertex].values())[0] / drone_v  # list(G[][].val) contains the distances
                asc = abs(linalg.norm(V[neighbour] - V[ve])) / drone_v
                tspan = np.array([t, t + dt])
                cc = 0
                if emergency_rerouting or True:
                    coll = [False and elem for elem in coll]

                    for j in range(nm):
                        d = minreldist(V[current_vertex], V[neighbour], j, tspan, mobs_obj, dummies_obj)
                        if d or d == 0:
                            coll[j] = True
                            cc = cc + (cmin + (1 - d) * (cmax - cmin))  # if d is not an empty list it will be handled like an int

                old_cost = cost[neighbour]
                new_cost = cost[current_vertex] + dt + cc + asc
                if new_cost < old_cost:
                    pq.put((new_cost, neighbour))
                    cost[neighbour] = new_cost
                    prev[neighbour] = current_vertex
                    tt[neighbour] = tt[current_vertex] + dt
                    collmat[neighbour, :] = coll

    vk = ve
    k = nv - 1
    route = np.zeros(nv)
    while vk != vs:  # Route building
        route[k] = vk
        k = k - 1
        vk = int(prev[vk])
    route[k] = vk

    route = route.astype(int)
    route = route[k:]  # Delete the zeros
    ttime = tt[route]
    collmat = collmat[route, :]

    if len(route) == 1:
        sys.exit("Path could not be created. \nThe two selected points should be in a same space.")
    return route, ttime, collmat


# ......................................................................................................................
def minreldist(v1, v2, j, tspan, mobs_obj, dummmies_obj):
    v1 = np.array([v1])
    v2 = np.array([v2])

    if j < len(mobs_obj):
        tgrid = mobs_obj[j].tgrid
        objC = mobs_obj[j].C
        if len(v1[0]) == 2:
            objC = objC[:, :2]   # 2D
        r = mobs_obj[j].r
        dummy = False
    else:
        j = j - len(mobs_obj)
        tgrid = dummmies_obj[j].tgrid
        objC = dummmies_obj[j].C
        if len(v1[0]) == 2:
            objC = objC[:, :2]       # 2D
        r = dummmies_obj[j].r
        dummy = True
    """
    premade = True
    if not premade:
        tgrid = make_tgrid(tspan[0], tspan[1], 0)
        C = make_C(mobs_obj, tgrid, j)
        alpha = make_alpha(C, v1, v2)
        Cp = make_Cp(v1, v2, alpha)
        d = make_d(C, Cp)
    else:
    """
    C = make_tC(tspan[0], tspan[1], tgrid, objC)
    if dummy:
        C_min, C_max, v_min, v_max = min_max(C, v1, v2, r, dummy)
        if v_max[0] > C_min[0] and v_min[0] < C_max[0] and \
                v_max[1] > C_min[1] and v_min[1] < C_max[1]:                                # 2D
            if v1[0, 0]-v2[0, 0] == 0 and v1[0, 1]-v2[0, 1] == 0:  # vertical v1->v2
                try:
                    d = make_d(C[:, :2], v1[:, :2])
                except TypeError:
                    d = make_d([C[0][:2]], v1[:,:2])
            else:
                try:
                    alpha = make_alpha(C[:, :2], v1[:, :2], v2[:, :2])
                    Cp = make_Cp(v1[:, :2], v2[:, :2], alpha)
                    d = make_d(C[:, :2], Cp)
                except TypeError:
                    alpha = make_alpha([C[0][:2]], v1[:, :2], v2[:, :2])
                    Cp = make_Cp(v1[:, :2], v2[:, :2], alpha)
                    d = make_d([C[0][:2]], Cp)
        elif v_max[0] > C_min[0] and v_min[0] < C_max[0] and \
                v_max[1] > C_min[1] and v_min[1] < C_max[1] and \
                v_min[2] < C_max[2]:                                  # TODO: 3D line-circle
            d = [100, 100]
            print("Head shot")
        else:
            d = [100, 100]
    else:
        C_min, C_max, v_min, v_max = min_max(C, v1, v2, r, dummy)
        if v_max[0] > C_min[0] and v_min[0] < C_max[0] and \
                v_max[1] > C_min[1] and v_min[1] < C_max[1]: # and \
                #v_max[2] > C_min[2] and v_min[2] < C_max[2]:
            if v1[0][0] == v2[0][0] and v1[0][1] == v2[0][1] and len(v1[0]) == 2:
                d = linalg.norm(C - v1, axis=1, ord=2)
            else:
                alpha = make_alpha(C, v1, v2)
                Cp = make_Cp(v1, v2, alpha)
                d = make_d(C, Cp)
        else:
            d = [100, 100]

    if any(k <= 2*r for k in d):
        co = min(d / 2*r)
    else:
        co = []
    return co


def min_max(C, v1, v2, r, dummy):
    C_min = np.amin(C, axis=0)-r
    C_max = np.amax(C, axis=0)+r
    v_min = np.minimum(v1, v2)
    v_max = np.maximum(v1, v2)
    if dummy:          # dummies are supper tall
        pass
        #C_max[2] = 10  # 3D
        #C_min[2] = 0
    return C_min, C_max, v_min[0], v_max[0]


def make_tC(tspan_0, tspan_1, tgrid, objC):
    if tgrid[-1] < tspan_0:
        C = [objC[-1]]
    else:
        C = objC
        remove = []
        for i in range(len(tgrid)):
            if tspan_0 > tgrid[i] or tgrid[i] > tspan_1:
                remove.append(i)
        C = np.delete(C, remove, axis=0)
    return C


def make_tgrid(tspan_0, tspan_1, mode):
    if mode == 0:
        tgrid = np.transpose(np.linspace(tspan_0, tspan_1, 10))
    if mode == 1:
        tgrid = np.transpose(np.arange(tspan_0, tspan_1, 0.05))
    return np.reshape(tgrid, tgrid.size)


def make_C(mobs_obj, tgrid, j):
    C = interpolate.splev(mobs_obj[j].move(tgrid), mobs_obj[j].path)
    C = np.array(C)
    return np.transpose(C)


def make_alpha(C, v1, v2):
    dist = linalg.norm(v2 - v1)
    #alpha = ((C - v1).dot(np.transpose(v2 - v1))) / (dist ** 2)
    alpha = np.divide(((C - v1).dot(np.transpose(v2 - v1))), (dist ** 2))
    alpha = [max(min(x, 1), 0) for x in alpha]
    return np.reshape(np.array([float(i) for i in alpha]), (-1, 1))


def make_Cp(v1, v2, alpha):
    Cp = []
    for i in range(len(v1[0])):
        Cp.append((v1[0][i] + (v2[0][i] - v1[0][i]) * np.transpose(alpha))[0])
    return np.transpose(np.array(Cp))


def make_d(C, Cp):
    return linalg.norm(C - Cp, axis=1, ord=2)


# ======================================================================================================================
def route_simplifier(route, fobs, box, bmm, inp_):
    # ADJ_graph
    route = route.tolist()
    idx = 0
    graph_adj = {}
    drone_r = inp_[8]
    drones_SE = inp_[9]
    V = inp_[1]
    obs_pos = copy.deepcopy(fobs[:, 0:3])
    obs_r = copy.deepcopy(fobs[:, -1])

    for i in drones_SE:
        for j in i:
            obs_pos = np.append(obs_pos, V[j])
            obs_r = np.append(obs_r, 2*drone_r)
    obs_pos = obs_pos.reshape(-1, 3)

    for i in route:
        graph_adj[i] = route[idx+1:]
        graph_adj[i] = [e for e in graph_adj[i] if not intersect(V[i], V[e], obs_pos, obs_r, box, bmm)]
        idx = idx+1

    R = nx.Graph()  # Adding the vertices to the graph
    for i in route:
        R.add_node(i, pos=[V[i]])

    for i in graph_adj:  # Adding the weighted edges to the graph
        for j in graph_adj[i]:
            R.add_edge(i, j, weight=abs(linalg.norm(V[i] - V[j])))

    inp_[0] = R
    rtc = dijkstra_timed(inp_)
    return rtc


# ======================================================================================================================
def flight_sim_3D(mobs_obj, dummies_obj, drone_obj, Ts, drone_numb, wanderer_obj):  # 2. input: animation frame delay in milliseconds
    fig = plt.gcf()  # Catch the current fig which previously created by create_view
    ax = fig.gca()
    u = np.linspace(0, 2 * np.pi, 7)
    v = np.linspace(0, np.pi, 7)
    plot = []
    xm, ym, zm = [], [], []
    ghost = False
    measurement_plot = False
    ax.view_init(elev=89.9, azim=269.9)

    # Measurement plot to simulation
    if measurement_plot:
        for i in range(3):
            pickle_in = open("Drone_positions"+"{:}".format(1)+".00000.pickle", "rb")
            measured_pos = pickle.load(pickle_in)
            pickle_in.close()
            measured_pos_0 = np.array(measured_pos[0])
            measured_pos_1 = np.array(measured_pos[1])
            measured_pos_2 = np.array(measured_pos[2])
            measured_pos_3 = np.array(measured_pos[3])
            plt.plot(measured_pos_0[:,1],measured_pos_0[:,2],measured_pos_0[:,3], 'black', lw=0.5, alpha=1)
            plt.plot(measured_pos_1[:,1], measured_pos_1[:,2], measured_pos_1[:,3], 'black', lw=0.5, alpha=1)
            plt.plot(measured_pos_2[:,1], measured_pos_2[:,2], measured_pos_2[:,3], 'black', lw=0.5, alpha=1)
            plt.plot(measured_pos_3[:,1], measured_pos_3[:,2], measured_pos_3[:,3], 'black', lw=0.5, alpha=1)

    for i in range(0, len(mobs_obj)-(drone_numb)):
        xm.append(mobs_obj[i].r * np.outer(np.cos(u), np.sin(v)))
        ym.append(mobs_obj[i].r * np.outer(np.sin(u), np.sin(v)))
        zm.append(mobs_obj[i].r * np.outer(np.ones(np.size(u)), np.cos(v)))
        plot.append([ax.plot_surface(xm[i], ym[i], zm[i], cmap=cm.Blues, alpha=0.6)])

    for i in range(len(mobs_obj)-(drone_numb), len(mobs_obj)):
        xm.append(mobs_obj[i].r * np.outer(np.cos(u), np.sin(v)))
        ym.append(mobs_obj[i].r * np.outer(np.sin(u), np.sin(v)))
        zm.append(mobs_obj[i].r * np.outer(np.ones(np.size(u)), np.cos(v)))
        plot.append([ax.plot_surface(xm[i], ym[i], zm[i], cmap=cm.Blues, alpha=1)])

    for i in range(0, len(dummies_obj)):
        z = np.linspace(0, dummies_obj[i].h, 7)
        theta_grid, z_grid = np.meshgrid(u, z)
        x_grid = dummies_obj[i].r * np.cos(theta_grid)
        y_grid = dummies_obj[i].r * np.sin(theta_grid)
        xm.append(x_grid)
        ym.append(y_grid)
        zm.append(z_grid)
        plot.append([ax.plot_surface(xm[i], ym[i], zm[i], cmap=cm.viridis, alpha=0.6)])

    wanderer_r = []
    for i in range(0, len(wanderer_obj)):
        wanderer_r.append(wanderer_obj[i].r)
        xm.append(wanderer_obj[i].r * np.outer(np.cos(u), np.sin(v)))
        ym.append(wanderer_obj[i].r * np.outer(np.sin(u), np.sin(v)))
        zm.append(wanderer_obj[i].r * np.outer(np.ones(np.size(u)), np.cos(v)))
        plot.append([ax.plot_surface(xm[i], ym[i], zm[i], cmap='copper', alpha=0.6)])

    xd = drone_obj[0].r * np.outer(np.cos(u), np.sin(v))
    yd = drone_obj[0].r * np.outer(np.sin(u), np.sin(v))
    zd = drone_obj[0].r * np.outer(np.ones(np.size(u)), np.cos(v))
    if ghost:
        plot.append([ax.plot_surface(xd, yd, zd, cmap=cm.Blues, alpha=0.8)])
    #plot.append([ax.plot_surface(xd, yd, zd, cmap=cm.Blues, alpha=1)])

    def update(frame_number, plot, xm, ym, zm, xd, yd, zd):

        # Wanderers (Moving obsticle with unkown movement)
        wanderer_pos = {}
        for i in range(len(mobs_obj) + len(dummies_obj), len(mobs_obj) + len(dummies_obj) + len(wanderer_obj)):
            plot[i][0].remove()
            j = i - (len(mobs_obj) + len(dummies_obj))
            x, y, z = wanderer_obj[j].move(frame_number * Ts * 0.001)
            xa = xm[i] + x
            ya = ym[i] + y
            za = zm[i] + z
            plot[i][0] = ax.plot_surface(xa, ya, za, cmap="copper", alpha=0.8)
            wanderer_pos[j] = [x, y, z]
        # Dummies
        for i in range(len(mobs_obj), len(mobs_obj) + len(dummies_obj)):
            plot[i][0].remove()
            j = i - len(mobs_obj)
            x, y, z = interpolate.splev(dummies_obj[j].move(frame_number * Ts * 0.001), dummies_obj[j].path)
            x = xm[i] + x
            y = ym[i] + y
            z = zm[i] #+ z
            plot[i][0] = ax.plot_surface(x, y, z, cmap="viridis", alpha=0.8)
        # Mobs
        for i in range(0, len(mobs_obj)-(drone_numb)):
            plot[i][0].remove()
            x, y, z = interpolate.splev(mobs_obj[i].move(frame_number * Ts * 0.001), mobs_obj[i].path)
            x = xm[i] + x
            y = ym[i] + y
            z = zm[i] + z
            plot[i][0] = ax.plot_surface(x, y, z, cmap="viridis", alpha=0.8)
        # Drones (prev)
        drone_pos = {}
        drone_colors = ['Reds', 'Blues', 'Greens', 'Wistia', 'Reds',
                        'YlOrBr', 'YlOrRd', 'OrRd', 'PuRd', 'RdPu', 'BuPu',
                        'GnBu', 'PuBu', 'YlGnBu', 'PuBuGn', 'BuGn', 'YlGn']
        for i in range(len(mobs_obj)-(drone_numb), len(mobs_obj)):
            plot[i][0].remove()
            x, y, z = interpolate.splev(mobs_obj[i].move(frame_number * Ts * 0.001), mobs_obj[i].path)
            xa = xm[i] + x
            ya = ym[i] + y
            za = zm[i] + z
            plot[i][0] = ax.plot_surface(xa, ya, za, cmap=drone_colors[i-(len(mobs_obj)-(drone_numb))])
            drone_pos[i-(len(mobs_obj)-(drone_numb))] = [x,y,z]
        # Ghost (v = const)
        if ghost:
            plot[-2][0].remove()
            x, y, z = interpolate.splev(drone_obj[0].move(frame_number * Ts * 0.001), drone_obj[0].path)
            x = xd + x
            y = yd + y
            z = zd + z
            plot[-2][0] = ax.plot_surface(x, y, z, cmap="magma", alpha=0.5)
        # Drone
        #plot[-1][0].remove()
        #x, y, z = interpolate.splev(drone_obj[0].gurobi_move(frame_number * Ts * 0.001), drone_obj[0].path)
        #xa = xd + x
        #ya = yd + y
        #za = zd + z
        #plot[-1][0] = ax.plot_surface(xa, ya, za, cmap="winter")
        #drone_pos[i - (len(mobs_obj) - (drone_numb))+1] = [x, y, z] # i is generated in the previous (Drones) for cicle

        # Stream the position data for the path checker
        # Optitrack like simulation
        frame_time = time.time()
        sim_data = {'Frame': frame_number, 'Time': frame_time, 'Drone_positions': drone_pos,
                    'Wanderer_positions': wanderer_pos, 'Wanderer_radius': wanderer_r}
        pickle_out = open("sim_data.pickle", "wb")  # Save drone_pos
        pickle.dump(sim_data, pickle_out)
        pickle_out.close()

    return FuncAnimation(fig, update, fargs=(plot, xm, ym, zm, xd, yd, zd), interval=Ts, blit=False)


# ======================================================================================================================
def gurobi_planner(drone_obj, mobs_obj, dummies_obj, input_dijkstra, fobs, box, bmm):
    Ts = 0.1
    Buff_list = [1, 2, 3, 4, 5]
    bi = 0
    emergency = False
    while bi < len(Buff_list):
        H = math.ceil(((drone_obj[0].len / drone_obj[0].v) / Ts) * Buff_list[bi])
        tgrid = np.arange(0, H * Ts, Ts)
        sgrid = np.linspace(0, drone_obj[0].len, 500)
        drone_pos = interpolate.splev(sgrid, drone_obj[0].path)
        table = np.zeros((len(tgrid), 101))
        table[:, 0] = np.transpose(tgrid)
        cntr = 0

        for i in range(0, len(mobs_obj)): # Mobs
            active = False
            mob_pos = interpolate.splev(mobs_obj[i].move(tgrid), mobs_obj[i].path)
            mob_pos = np.transpose(mob_pos)
            for j in range(0, len(tgrid)):
                if emergency:
                    svals = sgrid[linalg.norm(np.transpose(drone_pos) - mob_pos[j], axis=1, ord=2) < (drone_obj[0].r + mobs_obj[i].r)] # 3D
                else:
                    svals = sgrid[linalg.norm(np.transpose(drone_pos)[:, :2] - mob_pos[j][:2], axis=1, ord=2) < (drone_obj[0].r + mobs_obj[i].r)] # 2D
                if svals.any():
                    if not active:
                        active = True
                        cntr = cntr + 1
                    table[j, 2 * cntr - 1], table[j, 2 * cntr] = np.amin(svals), np.amax(svals)
                else:
                    active = False
        for i in range(0, len(dummies_obj)): # Dummies
            active = False
            mob_pos = interpolate.splev(dummies_obj[i].move(tgrid), dummies_obj[i].path)
            mob_pos = np.transpose(mob_pos)
            for j in range(0, len(tgrid)):
                svals = sgrid[linalg.norm(np.transpose(drone_pos)[:, :2] - mob_pos[j, :2], axis=1, ord=2) < (drone_obj[0].r + dummies_obj[i].r)]# and [np.transpose(drone_pos)[:, 2] <= mob_pos[j, 2]]] # TODO: h+
                if svals.any():
                    if not active:
                        active = True
                        cntr = cntr + 1
                    table[j, 2 * cntr - 1], table[j, 2 * cntr] = np.amin(svals), np.amax(svals)
                else:
                    active = False

        opt_mod = Model(name="linear program")
        opt_mod.setParam('OutputFlag', False)

        a = opt_mod.addVars(H, name='a', vtype=GRB.CONTINUOUS, lb=-drone_obj[0].a_max, ub=drone_obj[0].a_max)
        v = opt_mod.addVars(H + 1, name='v', vtype=GRB.CONTINUOUS, lb=0, ub=drone_obj[0].v_max)
        s = opt_mod.addVars(H + 1, name='s', vtype=GRB.CONTINUOUS)
        adir = opt_mod.addVars(cntr, name='adir', vtype=GRB.BINARY)
        cost_binary = opt_mod.addVars(H + 1, name='c_binary', vtype=GRB.BINARY)

        sB = drone_obj[0].len + 1

        opt_mod.addConstr(s[0] == 0)
        opt_mod.addConstr(s[len(s) - 1] == drone_obj[0].len)
        opt_mod.addConstr(v[0] == 0)
        opt_mod.addConstr(v[len(v)-1] == 0)
        opt_mod.addConstrs(v[k + 1] == v[k] + a[k] * Ts
                           for k in range(H))
        opt_mod.addConstrs(s[k + 1] == s[k] + v[k] * Ts + 0.5 * (Ts ** 2) * a[k]
                           for k in range(H))
        opt_mod.addConstrs(s[k] - adir[obs] * sB <= table[k, 2 * obs + 1]
                           for k in range(H)
                           for obs in range(cntr)
                           if table[k, 2 * obs + 2] > 0)
        opt_mod.addConstrs(s[k] + (1 - adir[obs]) * sB >= table[k, 2 * obs + 2]
                           for k in range(H)
                           for obs in range(cntr)
                           if table[k, 2 * obs + 2] > 0)

        #https: // math.stackexchange.com / questions / 2500415 / how - to - write - if - else -statement - in -linear - programming
        bigM = 1000
        opt_mod.addConstrs(s[k] + 0.0001 - bigM * (1 - cost_binary[k]) <= drone_obj[0].len
                           for k in range(H + 1))

        opt_mod.addConstrs(s[k] + bigM * cost_binary[k] >= drone_obj[0].len
                           for k in range(H + 1))

        tgrid_H = np.append(tgrid, H)
        s_opt = [min(drone_obj[0].v * t, drone_obj[0].len) for t in tgrid_H]
        # s_opt = [(drone_obj[0].v * t) for t in tgrid_H]

        # Opt with J1*v + J2*p
        d_p = [(s[x] - s_opt[x]) for x in range(len(s))]
        d_p = np.array(d_p)
        d_p = d_p.dot(np.transpose(d_p))
        d_v = [(v[x] - drone_obj[0].v) for x in range(len(v))]
        d_v = np.array(d_v)
        d_v = d_v.dot(np.transpose(d_v))
        J_p = 5
        J_v = 1
        d = J_p * d_p + J_v * d_v
        d_compensation = 0
        for x in range(len(v)):
            d_compensation += drone_obj[0].v ** 2 * (1 - cost_binary[x])
        opt_mod.setObjective(d - J_v * d_compensation)
        """
        d = [(v[x] - drone_obj[0].v) for x in range(len(v))]
        d = np.array(d)
        d = d.dot(np.transpose(d)) / H
        d_compensation = 0
        for x in range(len(v)):
            d_compensation += drone_obj[0].v**2 / H * (1 - cost_binary[x])
        opt_mod.setObjective(d - d_compensation)
        """
        opt_mod.ModelSense = GRB.MINIMIZE
        opt_mod.optimize()

        try:
            ans = opt_mod.objVal
            print('Buff =', Buff_list[bi])
            break
        except AttributeError as e:
            if bi == len(Buff_list)-1:
                print("Waiting drone blocking the way. Generating new route")
                input_dijkstra[7] = True
                rtc = dijkstra_timed(input_dijkstra)
                route = rtc[0]
                route_simplifier(route, fobs, box, bmm, input_dijkstra)
                route = rtc[0]
                x = []
                y = []
                z = []
                g = []
                V = input_dijkstra[1]
                for point in route:
                    g.append(V[point])
                for point in g:
                    x.append(point[0])
                    y.append(point[1])
                    z.append(point[2])
                gu = []
                for i in range(len(g) - 1):
                    gu.extend(np.linspace(g[i], g[i + 1], 3))
                    gu = np.delete(gu, -1, axis=0)
                    gu = gu.tolist()
                gu.extend([g[-1]])
                plt.plot(x, y, z, 'purple', lw=1.5, alpha=0.5)
                path = Path_creator(np.array(gu), "norm")
                X_path, Y_path, Z_path = path
                plt.plot(X_path, Y_path, Z_path, 'black', lw=1.5, alpha=1)
                path_tck, path_u = Path_creator(np.array(gu), "csnp")
                drone_obj[0].path = path_tck
                drone_obj[0].len = path_u[-1]
                bi = 0
                emergency = True
            else:
                bi += 1



    s_opt = [var.x for var in opt_mod.getVars() if "s" in var.VarName]
    s_opt = np.array(s_opt)
    if tgrid.shape != s_opt.shape:
        tgrid = np.append(tgrid, tgrid[-1] + Ts)
    spl = interpolate.splrep(tgrid, s_opt, k=2, s=0.001)
    drone_obj[0].gp = spl
    drone_obj[0].gt = tgrid[-1]

    results = Results()
    results.s = s_opt
    results.v = [var.x for var in opt_mod.getVars() if "v" in var.VarName]
    results.a = [var.x for var in opt_mod.getVars() if "a" in var.VarName]
    results.tgrid = tgrid

    return results
