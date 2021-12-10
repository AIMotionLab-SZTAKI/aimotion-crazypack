from casadi import SX, MX, Function, vertcat, sin, cos
from .frenet_path import FrenetPath
from matplotlib.patches import Polygon
import numpy as np
from numpy import interp
from .environment import Environment
class Obstacle(Environment):
    def __init__(self, ID : int = 0, corners: list = [], x_limits: list = [-0.2, 0.2], y_limits: list = [-0.3, 0.3], obstacle_type : str = "wall", generate_urdf : bool = True):
        super().__init__()
        """ Currently 3 types of obstacles will be implemented:
            - pole
            - wall
            - torus
        pole: A pole obstacle can be created with a single [x, y] coordinate, passed in
        using the corners arguement, but turly, it will be the center of the obstacle.
        From the input the actual corners are calculated using the size variable, which is hard-coded.

        wall: most basic, receives 4 corners

        torus: here we cheat a little. The input is basically a list of 4 corners. However, instead o
        avoding the wall these corneres represent, we take the first 2 corners, and extend it to the
        -y direction. Also, we take the second 2 corners and extend it to the +y direction.
        We create a torus-visualizing urdf file, and we return TWO wall objects. Thiks way the optimization can
        handle it such that as a result it will find a path, that goes through the torus.
        TODO: do we need to request two separate IDs?
        """


        self.ID = ID
        self.size = 0.1



        if obstacle_type == "wall":
            self.corners = corners
            if generate_urdf:
                self.generate_urdf_wall()
        elif obstacle_type == "pole":
            if corners:
                # if only the middle point is procived
                if len(corners) == 2:
                    # Don't know why I thought this was a good idea...
                    # self.corners = self.random_placement([corners[0], corners[0]], [corners[1], corners[1]])
                    # This is what I wanted:

                    size = self.size
                    delta = size/2
                    middle_x = corners[0]
                    middle_y = corners[1]
                    corner1 = [middle_x - delta, middle_y - delta]
                    corner2 = [middle_x + delta, middle_y - delta]
                    corner3 = [middle_x + delta, middle_y + delta]
                    corner4 = [middle_x - delta, middle_y + delta]
                    self.corners = [corner1] + [corner2] + [corner3] + [corner4]

                if generate_urdf:
                    self.generate_urdf_pole()
                    # self.generate_urdf_wall()
            else:
                # if random placement (we only allow random placement for poles)
                self.obstacle_type = "pole"
                self.corners = self.random_placement(self.obstacle_area["x_limits"], self.obstacle_area["y_limits"])
                if generate_urdf:
                    self.generate_urdf_pole()
                    # self.generate_urdf_wall()
        elif obstacle_type == "torus":
            # We are not allowed to do what I wanted to do. So let's do it some other way...
            self.corners = corners
            # if generate_urdf:
            #     self.generate_urdf_torus(corners1 = corners[0], corners2 = corners[1])
            #     assert "check return"
            #     return [Obstacle(ID = ID, corners=corners[0], obstacle_type="wall", generate_urdf=False),
            #             Obstacle(ID = ID, corners=corners[1], obstacle_type="wall", generate_urdf=False)]
        else:
            NotImplementedError()



        # Calculating center
        # self.corners = np.where(np.array(self.corners) < self.border_y[0], self.border_y[0], self.corners).tolist()
        # self.corners = np.where(np.array(self.corners) > self.border_y[1], self.border_y[1], self.corners).tolist()
        # Truncating corners
        for i in range( np.array(self.corners).reshape(4, -1).shape[0]  ):
            # x
            if self.corners[i][0] < self.border_x[0]:
                self.corners[i][0] = self.border_x[0]
            if self.corners[i][0] > self.border_x[1]:
                self.corners[i][0] = self.border_x[1]
                
            # y
            if self.corners[i][1] < self.border_y[0]:
                self.corners[i][1] = self.border_y[0]
            if self.corners[i][1] > self.border_y[1]:
                self.corners[i][1] = self.border_y[1]
                
                
        points = self.corners
        center = []
        center += [np.mean(np.array(points)[:, 0])]
        center += [np.mean(np.array(points)[:, 1])]
        self.center = center
        self.dx = np.abs(center[0] - self.corners[0][0])
        self.dy = np.abs(center[1] - self.corners[0][1])

        self.grid_world()
        
        # Calculating the 4 hyperplanes
        # down
        # mean of down corners
        # face 1
        center_to_mean = []
        b = []
        x_mean = np.mean(np.array(self.corners)[0:2, 0])
        y_mean = np.mean(np.array(self.corners)[0:2, 1])
        tmp = np.array([x_mean, y_mean]) - self.center
        center_to_mean += [tmp /  np.linalg.norm(tmp)]
        # b += [np.linalg.norm([x_mean, y_mean])]
        b += [center_to_mean[-1][0] * x_mean + center_to_mean[-1][1] * y_mean]
        # face 2
        x_mean = np.mean(np.array(self.corners)[1:3, 0])
        y_mean = np.mean(np.array(self.corners)[1:3, 1])
        tmp = np.array([x_mean, y_mean]) - self.center
        center_to_mean += [tmp /  np.linalg.norm(tmp)]
        # b += [np.linalg.norm([x_mean, y_mean])]
        b += [center_to_mean[-1][0] * x_mean + center_to_mean[-1][1] * y_mean]
        # face 3
        x_mean = np.mean(np.array(self.corners)[2:, 0])
        y_mean = np.mean(np.array(self.corners)[2:, 1])
        tmp = np.array([x_mean, y_mean]) - self.center
        center_to_mean += [tmp /  np.linalg.norm(tmp)]
        # b += [np.linalg.norm([x_mean, y_mean])]
        b += [center_to_mean[-1][0] * x_mean + center_to_mean[-1][1] * y_mean]
        # face 4
        x_mean = np.mean([np.array(self.corners)[i, 0] for i in (3, 0)]  )
        y_mean = np.mean([np.array(self.corners)[i, 1] for i in (3, 0)]  )
        tmp = np.array([x_mean, y_mean]) - self.center
        center_to_mean += [tmp /  np.linalg.norm(tmp)]
        # b += [np.linalg.norm([x_mean, y_mean])]
        b += [center_to_mean[-1][0] * x_mean + center_to_mean[-1][1] * y_mean]
        
        
        self.a = center_to_mean
        self.b = b
        
        
        # if corners:
        #     self.corners = corners
        # else:
        #     self.corners = self.random_placement(self.obstacle_area["x_limits"], self.obstacle_area["y_limits"])


        # self.generate_urdf()
        # self.scale()

    def scaled_corners(self, proportion = 1, size = 0.08 * 2 * 1):
        """We can scale a scare by proportion or by size.
        For example: scaling by 2 will increase the distance of the
        corners from the center twofold.
        By increasing by size, the sides of the obstacle will increase
        their distance from the center by 'size'."""
        import math
        center = np.array(self.center)
        scaled_corners = []
        # Scaling by proportion
        for i, corner in enumerate(self.corners):
            center_to_corner = corner - center
            new_center_to_corner = center_to_corner * proportion
            self.corners[i] = (center + new_center_to_corner).reshape(1, -1).tolist()[0]

        # Scaling by size
        for i, corner in enumerate(self.corners):
            if i == 0:
                previous_corner = self.corners[-1]
            else:
                previous_corner = self.corners[i - 1]

            vector_1 = np.array(corner) - center
            unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
            # self.corners[i] = (   center + (vector_1 + unit_vector_1 * math.sqrt(size**2 + size**2))   ).reshape(1, -1).tolist()[0]
            scaled_corners += [(   center + (vector_1 + unit_vector_1 * math.sqrt(size**2 + size**2))   ).reshape(1, -1).tolist()[0]]
            # vector_2 = np.array(previous_corner) - center
            # # angle between these corners
            # unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
            # unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
            # dot_product = np.dot(unit_vector_1, unit_vector_2)
            # angle = np.arccos(dot_product)
            # theta  = math.pi - math.pi/2 - angle / 2
            # opposite = size
            # hypotenuse = opposite / math.sin(theta)

            # self.corners[i] = (center + (vector_1 + unit_vector_1 * hypotenuse) ).reshape(1, -1).tolist()[0]


        # import math
        # for corner in self.corners:
        #     cc_distance = (corner[0] - self.center[0])**2 + \
        #                     (corner[0] - self.center[0])**2
        #     cc_distance = math.sqrt(cc_distance)
            # center_to_corner =


        # return self
        return scaled_corners
    def random_placement(self, x_limits, y_limits):
        import random
        seed = self.ID
        random.Random(seed)
        random.seed(seed)
        middle_x = random.uniform(*x_limits)
        middle_y = random.uniform(*y_limits)


        delta = self.size/2
        corner1 = [middle_x - delta, middle_y - delta]
        corner2 = [middle_x + delta, middle_y - delta]
        corner3 = [middle_x + delta, middle_y + delta]
        corner4 = [middle_x - delta, middle_y + delta]

        corners = [corner1, corner2, corner3, corner4]

        return corners
    
    def environment_space(self, environment):
        # https://stackoverflow.com/questions/36399381/whats-the-fastest-way-of-checking-if-a-point-is-inside-a-polygon-in-python
        import numpy as np
        import matplotlib.path as mpltPath
        path = mpltPath.Path(self.scaled_corners())
        # path = mpltPath.Path(self.corners)
        resolution = len(environment)
        for i in range(len(environment)):
            for j in range(len(environment[0])):
                x = environment[i][j][0]
                y = environment[i][j][1]
                inside = path.contains_points([np.array([x, y])])[0] #, radius = 0.001)
                if inside == True:
                    objective = {}
                    objective['ObjBound'] = -100
                    objective['ObjBoundC'] = -100
                    objective['ObjVal'] = -100
                    environment[i][j][2] = objective
        return environment    

    def grid_world(self, maze = [[0] * 100 for i in range(100)]):
        ""
        # https://stackoverflow.com/questions/36399381/whats-the-fastest-way-of-checking-if-a-point-is-inside-a-polygon-in-python
        import numpy as np
        import matplotlib.path as mpltPath
        path = mpltPath.Path(self.scaled_corners())
        resolution_x = len(maze[0])
        resolution_y = len(maze)
        x_grid = np.linspace(self.border_x[0], self.border_x[1], resolution_x).tolist()
        y_grid = np.linspace(self.border_y[1], self.border_y[0], resolution_y).tolist()
        for i, y in enumerate(y_grid):
            for j, x in enumerate(x_grid):
                inside = path.contains_points([np.array([x, y])])[0] #, radius = 0.001)
                if inside == True:
                    maze[i][j] = 1

        # for row in maze:
        #     line = []
        #     for col in row:
        #       if col == 1:
        #         line.append("\u2588")
        #       elif col == 0:
        #         line.append(" ")
        #       elif col == 2:
        #         line.append(".")
        #     print("".join(line))
        return maze

    def plot_obstacle(self, ax):

        # Plotting of obstacle

        corners = np.array(self.corners)
        corners = np.vstack((corners, corners[0, :]))
        polygon = Polygon(corners, closed=True, fill=True, fc=(0,0,0,0.1), ec=(0,0,0,1), lw=0.5, zorder = 1)
        ax.add_patch(polygon)


    def generate_urdf_pole(self):
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
        points = self.corners
        center = []
        center += [np.mean(np.array(points)[:, 0])]
        center += [np.mean(np.array(points)[:, 1])]
        center += [0.1]
        scale = 1
        # Calculating center
        # center = []
        # x = (self.corners[2][0] + self.corners[3][0] + corners_other[0][0] + corners_other[1][0]) / 4
        # center += [x]
        # y = (self.corners[2][1] + self.corners[3][1] + corners_other[0][1] + corners_other[1][1]) / 4
        # center += [y]
        # z = 1
        # center += [z]
        # scale = (corners_other[0][1] - self.corners[2][1]) / 2

        # Assembling .urdf file
        urdf_text = urdf_part1 + str(center[0]) + " " + str(center[1]) + " " + str(center[2])  \
        + urdf_part2 + str(scale) + " " + str(scale) + " " + str(scale)  \
        + urdf_part3

        # Writing .urdf file
        filename = self.cwd + "/urdf/" + "pole" + str(self.ID) + ".urdf"
        f = open(filename, "w")
        f.write(urdf_text)
        f.close()


        return 0




    def generate_urdf_torus(self,corners_other):
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
      <origin rpy="0 1.5708 0" xyz=" """

        urdf_part2 = """ "/>
      <geometry>
				<mesh filename="torus.obj" scale=" """
        urdf_part3 = """ "/>
      </geometry>
       <material name="white">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
	 	<mesh filename="torus.obj" scale="0 0 0"/>
      </geometry>
    </collision>
  </link>
</robot>
"""
        # Calculating center
        center = []
        x = (self.corners[2][0] + self.corners[3][0] + corners_other[0][0] + corners_other[1][0]) / 4
        center += [x]
        y = (self.corners[2][1] + self.corners[3][1] + corners_other[0][1] + corners_other[1][1]) / 4
        center += [y]
        z = 1
        center += [z]
        scale = (corners_other[0][1] - self.corners[2][1]) / 2

        # Assembling .urdf file
        urdf_text = urdf_part1 + str(center[0]) + " " + str(center[1]) + " " + str(center[2])  \
        + urdf_part2 + str(scale) + " " + str(scale) + " " + str(scale)  \
        + urdf_part3

        # Writing .urdf file
        filename = self.cwd + "/urdf/" + "torus" + str(self.ID) + ".urdf"
        f = open(filename, "w")
        f.write(urdf_text)
        f.close()


        return 0

    def generate_urdf_wall(self):
        urdf_part1 = """<?xml version="1.0"?>
<robot name="block_2">
  <link name="block_2_base_link">
    <contact>
      <lateral_friction value="1.0"/>
      <spinning_friction value=".001"/>
    </contact>
    <inertial>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <mass value="0.02"/>
      <inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/>
    </inertial>
    <visual>
      <origin rpy="0 0 0" xyz=" """
        urdf_part2 = """ "/>
      <geometry>
        <box size=" """
        urdf_part3 = """ "/>
      </geometry>
      <material name="blockmat">
        <color rgba="0.3 0.3 0.3 1"/>
      </material>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0.0 0.0 0.0"/>
      <geometry>
        <box size="0.0 0.0 0.0"/>
      </geometry>
    </collision>
  </link>
</robot> """


        # Calculating center
        points = self.corners
        center = []
        center += [np.mean(np.array(points)[:, 0])]
        center += [np.mean(np.array(points)[:, 1])]
        center += [0.7]

        # Calculating dimensions
        # Assuming points stores a list of [x, y] values and that
        # [0]: bottom left, [1] bottom right, [2] top right, [3] top left corners
        _dx = points[0][0] - center[0]
        dx = points[1][0] - center[0]
        _dy = points[0][1] - center[1]
        dy = points[2][1] - center[1]

        x_dim = abs(_dx - dx)
        y_dim = abs(_dy - dy)
        z_dim = center[2] * 2

        # Assembling .urdf file
        urdf_text = urdf_part1 + str(center[0]) + " " + str(center[1]) + " " + str(center[2])  \
        + urdf_part2 + str(x_dim) + " " + str(y_dim) + " " + str(z_dim) \
        + urdf_part3

        # Writing .urdf file
        filename = self.cwd + "/urdf/" + "box" + str(self.ID) + ".urdf"
        f = open(filename, "w")
        f.write(urdf_text)
        f.close()

        return 0




# corners = [[-0.5, -2.0],
#           [-0.3, -2.0],
#           [-0.3, -0.25],
#           [-0.5, -0.25]]
# obs = Obstacle(corners = corners)
