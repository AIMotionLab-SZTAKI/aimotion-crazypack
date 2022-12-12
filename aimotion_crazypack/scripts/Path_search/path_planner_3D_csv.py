import sys

from scripts.Path_search.planner_tools_3D import *

#def main():
if __name__ == '__main__':
# ----------------------------------------------------------------------------------------------------------------------
    # ALL THE THINGS WITH ONE CAN PLAY:

    # MOVING OBSTACLES
    obs_paths = np.array([2])                   # Give the indexes of the paths for the moving obstacles
    mobs_r = np.array([0])                      # The radius of the mobs
    mobs_v = np.array([2])                      # Speeds of mobs

    # CHOREOGRAPHY OF THE DRONES
    Random_SE = False
# ----------------------------------------------------------------------------------------------------------------------
    # # ARGUMENT PARSING
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--predef_index', default=0, type=int)
    # ARGS = parser.parse_args()

# ----------------------------------------------------------------------------------------------------------------------
    # LOAD THE SCENE
    pickle_in = open("scene_file_3D.pickle", "rb")  # Loading the layout
    scene = pickle.load(pickle_in)
    pickle_in.close()

    pickle_in = open("xyz_file_3D.pickle", "rb")  # The required xy points for making the paths for the moving obstacles
    scene.mobs = pickle.load(pickle_in)              # are loaded to scene.mobs
    pickle_in.close()
# ----------------------------------------------------------------------------------------------------------------------
    # MOVING OBSTACLES
    idx = obs_paths

    mobs_obj = []                                # Initialize a list for the mob objects
    for i in range(len(idx)):                    # Create the moving obstacle objects
        j = idx[i]                               # Create the data required to make a path with length accurate breaks
        path_tck, path_u = Path_creator(scene.mobs[j], "csnp")
        if i <= 1 or i == 3:                     # bnf for mack and forth movement
            mobs_obj.append(Mobs(mobs_r[i], mobs_v[i], path_tck, path_u[-1], "bnf", [], [], [], []))
        else:                                    # res to restart from the start
            mobs_obj.append(Mobs(mobs_r[i], mobs_v[i], path_tck, path_u[-1], "res", [], [], [], []))  # res to restart from the start
        tgrid = make_tgrid(0, path_u[-1]/mobs_v[i], 1)
        tgrid = np.append(tgrid, path_u[-1]/mobs_v[i])
        C = make_C(mobs_obj, tgrid, i)
        mobs_obj[i].tgrid = tgrid
        mobs_obj[i].C = C

    # Saving moving obstacle path to csv
    for i in range(len(mobs_obj)):
        generating_traj_csv(mobs_obj, "mobs", i)
# ----------------------------------------------------------------------------------------------------------------------
    # WANDERING OBSTACLES
    wanderer_num = 0
    w_r = [0.1, 0.3]
    w_sp = [[0, 0, 1], [0, 0, 0.5]]
    w_sv = [-0.04, -0.03]
    w_so = [[110, 0], [70, 0]]
    w_rs = 400
    w_max = [0, 0]
    wanderer_obj = []
    for i in range(wanderer_num):
        wanderer_obj.append(Wanderer(w_r[i], w_sp[i], w_sv[i], w_so[i], w_rs, w_max,  scene.dims))

# ----------------------------------------------------------------------------------------------------------------------

    if Random_SE:
        # Random choreography made by secene_small_construction_3D
        pickle_in = open("predef_SE_3D.pickle", "rb")  # Loading the layout
        predef_SE = pickle.load(pickle_in)
        print(predef_SE)
        pickle_in.close()
    else:
        """
        # Defining the choreography for the cube
        predef_SE = [[[0, 15], [10, 5], [3, 4], [1, 6]]]
        predef_SE.append([[15, 2], [5, 7], [4, 15], [6, 10]])
        predef_SE.append([[2, 13], [7, 8], [15, 0], [10, 5]])
        predef_SE.append([[13, 5], [8, 13], [0, 8], [5, 0]])
        predef_SE.append([[5, 10], [13, 8], [8, 13], [0, 11]])
        predef_SE.append([[10, 0], [8, 10], [13, 3], [11, 1]])
        """
        """
        # Defining the choreography for the pancake
        predef_SE = [[[0, 10], [1, 6], [2, 7], [3, 9]]]
        predef_SE.append([[10, 1], [6, 11], [7, 8], [9, 2]])
        predef_SE.append([[1, 3], [11, 9], [8, 10], [2, 0]])
        predef_SE.append([[3, 7], [9, 5], [10, 4], [0, 6]])
        predef_SE.append([[7, 8], [5, 9], [4, 10], [6, 11]])
        predef_SE.append([[8, 0], [9, 1], [10, 2], [11, 3]])
        """
        #predef_SE = [[[3, 4]]]  # 0

        # Defining the choreography for the city
        predef_SE = [[[32, 36], [33, 37], [34, 38], [35, 39]]]      #0
        predef_SE.append([[36, 19], [37, 12], [38, 30], [39, 35]])  #1
        predef_SE.append([[19, 36], [12, 9], [30, 21], [35, 39]])   #2
        predef_SE.append([[36, 28], [9, 37], [21, 29], [39, 35]])   #3
        predef_SE.append([[28, 36], [37, 3], [29, 9], [35, 39]])   #4
        predef_SE.append([[36, 24], [3, 15], [9, 25], [39, 35]])   #5
        predef_SE.append([[24, 36], [15, 37], [25, 38], [35, 39]])  #6
        predef_SE.append([[36, 32], [37, 33], [38, 34], [39, 35]])  #7
        # to flip position:
        #predef_SE.append([[32, 31], [33, 28], [34, 24], [35, 40]])  # 8


    # Reading predef_index and writing land if we should land
    predef_index = pickle_read("predef_index")
    if predef_index <= len(predef_SE) - 1:
        pass
    # if we are running out of se positions, then reset predef_index to 0
    else:
        predef_index = 0
        pickle_write("land", True)

    predef_index = 7 # rewrite index to constant
# ----------------------------------------------------------------------------------------------------------------------
    # SET THE NUMER OF THE  DRONES
    drone_num = 4
    G = scene.G
    V = scene.V
    V0 = scene.V0
    create_view(scene, idx)  # Create the static scene (fobs, mob paths)

    graph_vis = False
    if graph_vis:
        fig = plt.gcf()  # Catch the current fig which previously created by create_view
        ax = fig.gca()
        #for i in range(len(V) - 1):
        #   ax.scatter(V[i][0], V[i][1], V[i][2], s=0.5, alpha=0.5, c='black')
        pos = nx.get_node_attributes(G, 'pos')
        for i, j in enumerate(G.edges()):
            x = np.array((pos[j[0]][0][0], pos[j[1]][0][0]))
            y = np.array((pos[j[0]][0][1], pos[j[1]][0][1]))
            z = np.array((pos[j[0]][0][2], pos[j[1]][0][2]))
            ax.plot(x, y, z, c='black', lw=0.5, alpha=0.2)

    SE_list = []
    for drone_idx in range(drone_num):
        try: # In case of the manualy defined predef index is too big
            vs, ve, drones_SE = select_SE(V, V0, drone_idx, SE_list, predef_SE[predef_index], True)
        except IndexError:
            sys.exit("The value of the 'predef_index' exceeds the number of choreography steps."
                     "\nPlease give a lower number or comment it out")
        SE_list.append([vs, ve, drones_SE])

# --------------------------------------------------------------------------------------------------------------
    # MOVING DUMMIES
    idx_dummy = np.array([3])  # The indexes of the mob paths
    dummies_r = np.array([0])  # The radius of the mobs
    dummies_v = np.array([0.7])  # Speeds of dummies
    dummies_h = np.array([2])  # Height of dumies
    dummies_obj = []  # Initialize a list for the mob objects

    for i in range(len(idx_dummy)):  # Create the moving obstacle objects
        j = idx_dummy[i]  # Create the data required to make a path with length accurate breaks
        points = []
        for k in scene.mobs[j]:
            points.append([k[0], k[1], dummies_h[i]])
        path_tck, path_u = Path_creator(points, "csnp")
        # bnf for back and forth movement
        dummies_obj.append(Dummies(dummies_r[i], dummies_h[i], dummies_v[i], path_tck, path_u[-1], [], [], [], []))

        tgrid = make_tgrid(0, path_u[-1] / dummies_v[i], 1)
        tgrid = np.append(tgrid, path_u[-1] / dummies_v[i])
        C = make_C(dummies_obj, tgrid, i)
        dummies_obj[i].tgrid = tgrid
        dummies_obj[i].C = C

    # Saving moving obstacle path to csv #TODO
    # for i in range(len(mobs_obj)):
    #    generating_traj_csv(mobs_obj, "mobs", i)

# ----------------------------------------------------------------------------------------------------------------------
    timer_allin_Start = time.time()
    real_mobs = copy.deepcopy(mobs_obj)
    all_save = [scene, dummies_obj, real_mobs]
    results = []
    for drone_idx in range(drone_num):
        vs, ve, drones_SE = SE_list[drone_idx]

    # ------------------------------------------------------------------------------------------------------------------
        # DIJKSTRA PATH PLANNER
        print("--------------------------------------------------------\nDrone:", drone_idx)
        print("Start:", vs, V[vs], "End:", ve, V[ve])
        drone_r = 0.12
        drone_v = np.array([0.6])
        drone_obj = []
        rtc = []                                     # Contains the results of the dijkstra (route, ttime, collmat)
        route = [None] * len(drone_v)                # Contains the vertexes which makes the route
        ttime = [None] * len(drone_v)                # Contains the travel time to each vertex
        collmat = [None] * len(drone_v)              # Contains the collision matrix
        route_pre = []                               # Contains the vertexes which makes the route
        ttime_pre = []

        multiprocess = False

        timer_start = time.time()
        if not multiprocess:
            "Single core computation"
            for i in range(len(drone_v)):
                input_values = [G, V, mobs_obj, dummies_obj, drone_v[i], vs, ve, False, drone_r, drones_SE]
                rtc = dijkstra_timed(input_values)
                route_pre.append(rtc[0])
                ttime_pre.append(rtc[1])
                rtc = route_simplifier(rtc[0], scene.fobs, scene.box, scene.boxminmax, input_values)
                route[i] = rtc[0]
                ttime[i] = rtc[1]
                collmat[i] = rtc[2]
            best = len(drone_v) - 1
            for i in range(len(drone_v)):  # Chose the velocity with the fastest time
                if ttime[i][-1] < ttime[best][-1]:
                    best = i

        else:
            "Or, we can do multiprocessing :))"
            import multiprocessing as mp
            target_function = dijkstra_timed

            input_values = []
            for i in range(len(drone_v)):
                input_values.append([G, V, mobs_obj, dummies_obj, drone_v[i], vs, ve, False, drone_r, drones_SE])

            with mp.Pool(len(drone_v)) as p:
                res = p.map(target_function, input_values)

            for i in range(len(drone_v)):
                rtc = res[i]
                route_pre.append(rtc[0])
                ttime_pre.append(rtc[1])

            best = len(drone_v)-1
            for i in range(len(drone_v)):                # Chose the velocity with the fastest time
                if ttime_pre[i][-1] < ttime_pre[best][-1]:
                    best = i

            input_values = input_values[best]
            rtc = route_simplifier(route_pre[best], V, scene.fobs, scene.box, scene.boxminmax, input_values)
            route[best] = rtc[0]
            ttime[best] = rtc[1]
            collmat[best] = rtc[2]

        timer_end = time.time()
        print("Chosen vel:", drone_v[best], "m/s",
              "\nRoute:\n", route_pre[best],
              "\nSimplified route:\n", route[best],
              "\nTime difference:", ttime[best][-1] - ttime_pre[best][-1], "sec",
              "\nTime of flight:\n", ttime[best],
              "\nCollision matrix:\n", collmat[best],
              "\nTime of A*:", timer_end - timer_start, "sec"
              )
    # ..................................................................................................................
        # plotting the best path and smoothing the spline with extra points
        visible_drone_path = 99

        x = []
        y = []
        z = []
        g = []

        x_pre = []
        y_pre = []
        z_pre = []
        g_pre = []

        # Plot the original route
        for point in route_pre[best]:
            g_pre.append(V[point])
        for point in g_pre:
            x_pre.append(point[0])
            y_pre.append(point[1])
            z_pre.append(point[2])
        if drone_idx == visible_drone_path or 99 == visible_drone_path:
            plt.plot(x_pre, y_pre, z_pre, 'blue', lw=0.5, alpha=0.5)

        # plot the simplified route
        for point in route[best]:
            g.append(V[point])
        for point in g:
            x.append(point[0])
            y.append(point[1])
            z.append(point[2])
        if drone_idx == visible_drone_path or 99 == visible_drone_path:
            plt.plot(x, y, z, 'green', lw=1.5, alpha=0.8)

        #Add a point in the midle of each edge to make the spline closer to the original edge
        gu = []
        for i in range(len(g) - 1):
            gu.extend(np.linspace(g[i], g[i + 1], 5))
            gu = np.delete(gu, -1, axis=0)
            gu = gu.tolist()
        gu.extend([g[-1]])

        path = Path_creator(np.array(gu), "norm")
        X_path, Y_path, Z_path = path
        if drone_idx == visible_drone_path or 99 == visible_drone_path:
            plt.plot(X_path, Y_path, Z_path, 'red', lw=1.5, alpha=1)
        else:
            plt.plot(X_path, Y_path, Z_path, 'brown', lw=0.8, alpha=0.8)

        path_tck, path_u = Path_creator(np.array(gu), "csnp")
        drone_obj.append(Drone(drone_r, drone_v[best], path_tck, path_u[-1], [], []))
    # ------------------------------------------------------------------------------------------------------------------
        #if drone_idx == 2: # Preploting for check paths if gurobi throw an error
        #    plt.show()

        # GUROBI
        timer_start = time.time()
        input_values[0] = G
        results.append(gurobi_planner(drone_obj, mobs_obj, dummies_obj, input_values, scene.fobs, scene.box, scene.boxminmax))
        timer_end = time.time()
        print("Time of gurobi:", timer_end - timer_start, "sec")

        x, y, z = interpolate.splev(drone_obj[0].gurobi_move(results[drone_idx].tgrid), drone_obj[0].path)

        t = results[drone_idx].tgrid
        print([t, x, y, z])
        # print("t hossz:" + t.shape + " tgrid hossz: " + tgrid.shape + " x hossz: " + x.shape)

        Pontok = list(map(list, zip(t.tolist(), x.tolist(), y.tolist(), z.tolist())))
        filenev="./boti_csv/Drone_" + str(drone_idx) + "_palya_"+ str(predef_index) + ".csv"
        with open(filenev, 'w', newline='') as f:
            writer=csv.writer(f)
            writer.writerows(Pontok)
        # -----------------------------------------------------------------------------------------------------------------
        # DRONE TO MOBS
        mobs_obj.append(Mobs(drone_r, "", drone_obj[0].path, "", "gurobi", "", "", drone_obj[0].gp, drone_obj[0].gt))
        tgrid = make_tgrid(0, drone_obj[0].gt, 1)
        tgrid = np.append(tgrid, drone_obj[0].gt)
        C = make_C(mobs_obj, tgrid, -1)

        mobs_obj[-1].tgrid = tgrid
        mobs_obj[-1].C = C
        # saving to csv
        generating_traj_csv(drone_obj, "vehicle", 0, drone_idx)

        all_save.append({'Drone_objects': drone_obj, 'Path_breaks': path_u})
    # ----------------------------------------------------------------------------------------------------------------------
    timer_allin_End = time.time()
    print("--------------------------------------------------------\n"
          "--------------------------------------------------------\n"
          "All time:", timer_allin_End-timer_allin_Start, "sec\n"
          "Move idx:", predef_index)
# ----------------------------------------------------------------------------------------------------------------------
    # Writing initial positions into yaml file:
    initial_positions = []
    # Because all drones are saved to mobs_obj as well:
    for i in range(len(mobs_obj)):
        if mobs_obj[i].mode == "gurobi":
            initial_positions.append(np.array(interpolate.splev(mobs_obj[i].move(0), mobs_obj[i].path)).reshape(-1,).tolist())

    write_initial_position_yaml(initial_positions)

    # increasing predef_index
    pickle_write("predef_index", predef_index + 1)

    # tell the checker there is new paths for the drones
    pickle_out = open("sim_start.pickle", "wb")
    pickle.dump(all_save, pickle_out)
    pickle_out.close()

#     # SIMULATE THE FLIGHT
    Ts = 50 # Change in the Path_checker too
    animation = flight_sim_3D(mobs_obj, dummies_obj, drone_obj, Ts, drone_num, wanderer_obj) # Animate the moving objects (mobs, drone)
    plt.show()                                                              # 2. input: animation frame delay in milliseconds

# ......................................................................................................................
    # velocity plot

    Plot_v = False
    if Plot_v:
        fig = plt.figure()
        tellme("Velocity")
        for i in range(len(results)):
            plt.plot(results[i].tgrid, results[i].v, lw=2)
            plt.plot([results[i].tgrid[0], results[i].tgrid[-1]], [drone_v[best], drone_v[best]], 'red', lw=2)
        plt.show()

"""
if __name__ == '__main__':
    profiler = cProfile.Profile()
    profiler.enable()
    main()
    profiler.disable()
    stats = pstats.Stats(profiler).sort_stats('cumtime')
    #stats.print_stats()
"""



