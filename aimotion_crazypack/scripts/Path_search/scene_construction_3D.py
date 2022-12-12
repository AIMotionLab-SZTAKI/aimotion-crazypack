from planner_tools_3D import *
from read_optitrack import read_optitrack
import pickle

#----------------------------------------------------------------------------------------------------------------------
if __name__ == '__main__':
    scene = Scene()                                             # Scene object is defined in the planner_tools.
                                                                # We use it to store base data.
    scene.dims = np.array([-1.4, 1.4, -1.4, 1.4, 0.4, 1.1])     # The size of the scene
    scene.fobs = np.array([[0, 0, 0, 0], [0, 0, 0, 0]])

    Real_obs = False
    show_graph = False

    if not Real_obs:
         # TODO: fix not real obst (add landing zones)
        scene.box = np.array([[[-0.25, -0.25, 0], [0.25, -0.25, 0], [0.25, 0.25, 0], [-0.25, 0.25, 0],
                               [-0.25, 0.25, 0.01], [-0.25, -0.25, 0.01], [0.25, -0.25, 0.01], [0.25, 0.25, 0.01]]])
        landingzone_positions = [0, 0, 0.5]
    else:
        # Adding box obstacles from file
        obstacle_positions = list(read_optitrack(filename='dijkstra3D/csv/optitrack.csv')[0].values())
        landingzone_positions_unordered = list(read_optitrack(filename='dijkstra3D/csv/optitrack.csv')[2].values())
        landingzone_indexes = list(read_optitrack(filename='dijkstra3D/csv/optitrack.csv')[2].keys())
        none_list = [None] * 15
        for i in range(len(landingzone_indexes)):
            none_list[int(landingzone_indexes[i][2:]) - 1] = landingzone_positions_unordered[i]
        landingzone_positions = list(filter(lambda item: item is not None, none_list))
        # obstacle_positions = []

        obst_dimensions = [0.19, 0.19, 1.6]
        boxes = []
        for obstacle in obstacle_positions:
            box_corners = []
                                                # adding corners in a counter-clockwise fashion for the bottom part. Then also counter-clockwise fashion for the top part.
            for corner_x, corner_y, corner_z in zip([-1, 1, 1, -1, -1, -1, 1, 1], [-1, -1, 1, 1, 1, -1, -1, 1], [-1, -1, -1, -1, 0, 0, 0, 0]):
                # center +/- obstacle size
                box_corners.append([obstacle["x"] + corner_x * obst_dimensions[0], -obstacle["z"] + corner_y * obst_dimensions[1], obstacle["y"] + corner_z * obstacle["y"]])
            boxes.append(box_corners)

        #landingpad_dimensions = [0.2, 0.2, 0.1]
        stick_dimensions = [0.27, 0.27]
        for obstacle in landingzone_positions:
            #box_corners = []

            # Landing pad
            #for corner_x, corner_y, corner_z in zip([-1, 1, 1, -1, -1, -1, 1, 1], [-1, -1, 1, 1, 1, -1, -1, 1], [-1, -1, -1, -1, 1, 1, 1, 1]):
            #    # center +/- obstacle size
            #    box_corners.append([obstacle["x"] + corner_x * landingpad_dimensions[0], -obstacle["z"] + corner_y * landingpad_dimensions[1], obstacle["y"] + corner_z * landingpad_dimensions[2]])
            #boxes.append(box_corners)

            box_corners = []
            # support stick
            for corner_x, corner_y, corner_z in zip([-1, 1, 1, -1, -1, -1, 1, 1], [-1, -1, 1, 1, 1, -1, -1, 1], [-1, -1, -1, -1, 0, 0, 0, 0]):
                 #center +/- obstacle size
                box_corners.append([obstacle["x"] + corner_x * stick_dimensions[0], -obstacle["z"] + corner_y * stick_dimensions[1], obstacle["y"] + corner_z * obstacle["y"]])
            boxes.append(box_corners)

        #room = [[[-6.9, -6.1, 0], [-4.0, -6.1, 0], [-6.5, 6.1, 0], [-6.9, 6.1, 0],
        #                 [-6.9, 6.1, 2], [-6.9, -6.1, 2], [-4.0, -6.1, 2], [-6.5, 6.1, 2]],
        #                [[-2.5, -4.5, 0], [-1.5, -4.5, 0], [-1.5, -3.5, 0], [-2.5, -3.5, 0],
        #                 [-2.5, -3.5, 2], [-2.5, -4.5, 2], [-1.5, -4.5, 2], [-1.5, -3.5, 2]],
        #                [[-2.5, -4.5 + 7, 0], [-1.5, -4.5 + 7, 0], [-1.5, -3.5 + 7, 0], [-2.5, -3.5 + 7, 0],
        #                 [-2.5, -3.5 + 7, 2], [-2.5, -4.5 + 7, 2], [-1.5, -4.5 + 7, 2], [-1.5, -3.5 + 7, 2]]]
        #for i in room:
        #    boxes.append(i)

        # Adding the boxes to the scene
        scene.box = np.array(boxes)

    for i in range(scene.box.shape[0]):
        p = []
        for j in range(8):
            p.append(GeoPoint(scene.box[i][j][0], scene.box[i][j][1], scene.box[i][j][2]))
        gp = [p[0], p[1], p[2], p[3],
              p[4], p[5], p[6], p[7]]
        gpInst = GeoPolygon(gp)
        scene.boxminmax.append(GeoPolygonProc(gpInst))

    if Real_obs:
        # Generating pole urdf
        for i in range(len(obstacle_positions)):
            pos = []
            pos.append(obstacle_positions[i]["x"])
            pos.append(-obstacle_positions[i]["z"])
            pos.append(0.1)
            generating_box_urdf(pos, i)
    
# ----------------------------------------------------------------------------------------------------------------------
    try:                                             # Open the xy_list if it exists
        pickle_in = open("xyz_file_3D.pickle", "rb")
        xy_list = pickle.load(pickle_in)
        scene.mobs = xy_list                        # Show all existing paths if uncommented
    except (OSError, IOError) as e:                  # Create an empty xy_list
        xy_list = []

    create_view(scene, [])
# ----------------------------------------------------------------------------------------------------------------------
    # CREATE THE GRAPH
    # Fix vertices in the graph ([x1, y1, z1], [x2, y2, z2], ...)
    # Predefined hover points
    """
    # Shape: o o o
    #        o   o  2 layer
    #        o o o
    fp = 1.2
    lh = 0.4
    hh = 1.1
    off_x = 0.2
    off_y = 0.2
    V0 = [[-fp+off_x, -fp+off_y, hh], [-fp+off_x, 0+off_y, hh], [-fp+off_x, fp+off_y, hh], [0+off_x, -fp+off_y, hh],
                   [0+off_x, fp+off_y, hh], [fp+off_x, -fp+off_y, hh], [fp+off_x, 0+off_y, hh], [fp+off_x, fp+off_y, hh],
                   [-fp, -fp, lh], [-fp, 0, lh], [-fp, fp, lh], [0, -fp, lh],
                   [0, fp, lh], [fp, -fp, lh], [fp, 0, lh], [fp, fp, lh]]
    """
    """
    # Shape: 0 o o 0
    #        o     o   0 are higher than o
    #        o     o
    #        0 o o 0
    xys = 0.4
    xyl = 3 * xys
    hh = 1.1
    lh = 0.4
    V0 = [[xyl, -xyl, hh], [xyl, -xys, lh], [xyl, xys, lh], [xyl, xyl, hh],
          [xys, -xyl, lh], [xys, xyl, lh], [-xys, -xyl, lh], [-xys, xyl, lh],
          [-xyl, -xyl, hh], [-xyl, -xys, lh], [-xyl, xys, lh], [-xyl, xyl, hh]]
    """

    # Shape: X_________________________
    #           03/19 02/18 01/17 00/16|
    #           07/23 06/22 05/21 04/20|   2 layer
    #           11/27 10/26 09/25 08/24|
    #           15/31 14/30 13/29 12/28|
    #                                  Y
    xys = 0.4
    xyl = 3 * xys
    hh = 1.1
    lh = 0.4
    ic = 0.15
    V0 = [[-xyl, -xyl, hh], [-xys, -xyl, hh], [xys, -xyl, hh], [xyl, -xyl, hh],
          [-xyl, -xys, hh], [-xys-ic, -xys-ic, hh], [xys+ic, -xys-ic, hh], [xyl, -xys, hh],
          [-xyl,  xys, hh], [-xys-ic,  xys+ic, hh], [xys+ic,  xys+ic, hh], [xyl,  xys, hh],
          [-xyl,  xyl, hh], [-xys,  xyl, hh], [xys,  xyl, hh], [xyl,  xyl, hh],
          [-xyl, -xyl, lh], [-xys, -xyl, lh], [xys, -xyl, lh], [xyl, -xyl, lh],
          [-xyl, -xys, lh], [-xys-ic, -xys-ic, lh], [xys+ic, -xys-ic, lh], [xyl, -xys, lh],
          [-xyl,  xys, lh], [-xys-ic,  xys+ic, lh], [xys+ic,  xys+ic, lh], [xyl,  xys, lh],
          [-xyl,  xyl, lh], [-xys,  xyl, lh], [xys,  xyl, lh], [xyl,  xyl, lh]]


    #stick_pos = get_stick_pos()
    #
    #V0.extend(stick_pos) # Stick pos are extra hover points (not prioritized points)

    # Hover points above landing zones
    if Real_obs:
        scene.L_zone_num = len(landingzone_positions)
        H = 0.2
        Landing_hover = []
        for j in range(scene.L_zone_num):
            i = j+len(obstacle_positions)
            x_ = (scene.boxminmax[i].x0+scene.boxminmax[i].x1)/2
            y_ = (scene.boxminmax[i].y0 + scene.boxminmax[i].y1) / 2
            z_ = max(scene.boxminmax[i].z1 + H, lh)
            V0.append([x_, y_, z_])

    V0 = np.array(V0)

    Nv = 200  # Maximal number of vertices to be generated
    thres = 0.1
    seed = 444
    timer_start = time.time()

    G, V = routing_graph(scene, V0, Nv, thres, seed, show_graph)
    timer_end = time.time()
    print("Routing graph time:", timer_end-timer_start, "sec")
    scene.G = G
    scene.V = V
    scene.V0 = V0
# ----------------------------------------------------------------------------------------------------------------------
    # ADD NEW MOB PATHS
    xy_list += get_path(xy_list, V0)
# ----------------------------------------------------------------------------------------------------------------------
    # RANDOM CHOREOGRAPHY
    choreo_steps = 5
    n_drones = 6
    cleaner_rand = True # the landing zones are not prioritized and
                         # the drones can't fly where the other drones start their movement

    drones = {}
    for i in range(n_drones):
        drones[i] = [i, None]
    random.seed(12)
    drone_num = len(drones)
    landing_number = min(len(landingzone_positions), drone_num)
    hover_points = list(np.arange(0, len(scene.V0) - len(landingzone_positions), 1))
    landing_points = list(np.arange(len(scene.V0)-len(landingzone_positions), len(scene.V0), 1))
    if cleaner_rand:
        hover_points.extend(landing_points)
        if 2 * n_drones > len(hover_points):
            sys.exit("Too many drones")
    predef_SE = []
    previous_landing = None

    for step in range(choreo_steps):
        drones_SE = []

        if cleaner_rand:
            print("------------------------------"
                  "\nStep:", step + 1)
            available_points = copy.deepcopy(hover_points)
            for i in range(len(drones)):              # remove the start points
                available_points.remove(drones[i][0])
            for i in drones:
                drones[i][1] = random.sample(list(available_points), 1)[0]
                available_points.remove(drones[i][1])
                drones_SE.append([drones[i][0], drones[i][1]])
                print("Drone:", i, "goes from:", drones[i][0], "to:", drones[i][1])
                drones[i][0] = drones[i][1]
            predef_SE.append(drones_SE)

        else:
            # Choose the landing drones
            landers = list(drones)
            if drones[0][1] or drones[0][1] == 0: # skip first
                if len(landing_points) == 1:
                    landers.remove(landing_drones[0])
            landing_drones = random.sample(landers, landing_number)
            print("------------------------------"
                  "\nStep:", step+1,
                  "\nLanding drone(s):", landing_drones)

            k = 0
            available_points_h = list(np.arange(0, len(scene.V0) - len(landingzone_positions), 1))
            available_points_l = list(np.arange(len(scene.V0)-len(landingzone_positions), len(scene.V0), 1))

            # Choreography maker
            for j in drones:
                removed = False
                print("\nDrone:", j)
                if any(t == j for t in landing_drones):
                    print(" Available landing points:", available_points_l)
                    if any(t == drones[j][0] for t in landing_points):
                        try:
                            available_points_l.remove(drones[j][0]) # remove curent poit to avoid second time
                        except:
                            removed = True
                    try:
                        drones[j][1] = random.sample(list(available_points_l), 1)[0]
                        if any(t == drones[j][0] for t in landing_points) and not removed:
                            available_points_l.append(drones[j][0])
                        available_points_l.remove(drones[j][1]) # remove future point to aviod collision
                        print("From:", drones[j][0], "Go to:", drones[j][1])
                    except ValueError:
                        drones[j][1] = random.sample(list(available_points_h), 1)[0]
                        available_points_h.remove(drones[j][1])  # remove future point to aviod collision
                        print("Can't stay here:", drones[j][0], "Go to:", drones[j][1])
                else:
                    print(" Available hover points:", available_points_h)
                    if any(t == drones[j][0] for t in hover_points):
                        try:
                            available_points_h.remove(drones[j][0])
                        except:
                            removed = True
                    drones[j][1] = random.sample(list(available_points_h), 1)[0]
                    if any(t == drones[j][0] for t in hover_points) and not removed:
                        available_points_h.append(drones[j][0])
                    available_points_h.remove(drones[j][1])
                    print("From:", drones[j][0], "Go to:", drones[j][1])
                drones_SE.append([drones[j][0], drones[j][1]])
                drones[j][0] = drones[j][1]
            predef_SE.append(drones_SE)

    print("=============================="
          "\nThe choreography:\n", predef_SE)
# ----------------------------------------------------------------------------------------------------------------------
    # SAVE OUT
    pickle_out = open("scene_file_3D.pickle", "wb")     # Save scene
    pickle.dump(scene, pickle_out)
    pickle_out.close()

    pickle_out = open("xyz_file_3D.pickle", "wb")        # Save xy_list
    pickle.dump(xy_list, pickle_out)
    pickle_out.close()

    pickle_out = open("predef_SE_3D.pickle", "wb")     # Save scene
    pickle.dump(predef_SE, pickle_out)
    pickle_out.close()
