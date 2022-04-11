import pickle
from planner_tools import *
import numpy as np
from read_optitrack import read_optitrack
import os
from os.path import exists
import matplotlib.pyplot as plt
from concurrent.futures import ProcessPoolExecutor, as_completed
import multiprocessing as mp
import random

sim = False


def main():
    os.system("mkdir dijkstra/csv")
    os.system("mkdir dijkstra/yaml")
    os.system("mkdir dijkstra/urdf")
    # ----------------------------------------------------------------------------------------------------------------------
    # LOAD THE SCENE
    pickle_in = open("scene_file.pickle", "rb")  # Loading the layout
    scene = pickle.load(pickle_in)
    pickle_in.close()

    pickle_in = open("xy_file.pickle", "rb")  # The required xy points for making the paths for the moving obstacles
    scene.mobs = pickle.load(pickle_in)  # are loaded to scene.mobs
    pickle_in.close()

    # Deleting the fixed obstacles
    scene.fobs = []
    # Adding box obstacles from file
    obstacle_positions = list(read_optitrack(filename='dijkstra/csv/optitrack.csv')[0].values())

    # In case we want to add boxes randomly:
    n_random_boxes = 0
    for i in range(n_random_boxes):
        box_pos, _ = random_V0_generator(x_lim=scene.dims[:2], y_lim=scene.dims[2:], d_min=1.0, p0=None, d_max=1.5)
        pos_dict = {"x": box_pos[0], "z": -box_pos[1]}
        obstacle_positions.append(pos_dict)

    obst_size = 0.13
    boxes = []
    for obstacle in obstacle_positions:
        box_corners = []
        for corner_x, corner_y in zip([-1, 1, 1, -1], [-1, -1, 1, 1]):
            # center +/- obstacle size
            box_corners.append([obstacle["x"] + corner_x * obst_size, -obstacle["z"] + corner_y * obst_size])
        boxes.append(box_corners)

    # Adding the boxes to the scene
    scene.box = np.array(boxes)

    # Generating pole urdf
    for i in range(len(obstacle_positions)):
        pos = []
        pos.append(obstacle_positions[i]["x"])
        pos.append(-obstacle_positions[i]["z"])
        pos.append(0.1)
        generating_box_urdf(pos, i)

    # ----------------------------------------------------------------------------------------------------------------------
    # MOVING OBSTACLES
    idx = np.array([6, 1, 2])  # The indexes of the mob paths
    mobs_r = np.array([0.1, 0.1, 0.1]) * 0  # The radius of the mobs (TODO: make it accurate)
    mobs_v = np.array([1, 1.5, 0.5])  # Speeds of mobs
    mobs_obj = []  # Initialize a list for the mob objects

    for i in range(len(idx)):  # Create the moving obstacle objects
        j = idx[i]  # Create the data required to make a path with length accurate breaks
        path_tck, path_u = Path_creator(scene.mobs[j], "csnp")
        mobs_obj.append(Mobs(mobs_r[i], mobs_v[i], path_tck, path_u[-1], "bnf"))

    # ----------------------------------------------------------------------------------------------------------------------
    # CREATE THE GRAPH
    V0 = np.array([[0, 0], [0, 0], [0, 0]])  # Fix vertices in the graph ([x1, y1], [x2, y2], ...)
    create_view(scene, idx)  # Create the static scene (fobs, mob paths)

    # ----------------------------------------------------------------------------------------------------------------------

    # nx.draw_networkx(G, pos=V, with_labels=False, node_size=10, alpha=0.3)
    return scene, mobs_obj, V0


# --------------------------------------------------------------------------
def main2(scene, mobs_obj, V0):
    Nv = 300  # Maximal number of vertices to be generated
    thres = 0.1
    seed = 440

    time_graph_S = time.time()
    G, V = routing_graph(scene, V0, Nv, thres, seed)
    time_graph_E = time.time()
    # DIJKSTRA PATH PLANNER
    drone_r = 0.08  # actually it is 0.08, but we set it to 0.1 to have some safety margin.
    drone_v = np.array([1, 1.5, 2])
    # drone_v = np.array([1])
    # drone_v = np.linspace(0.5, 1.5, 8)
    drone_obj = []
    rtc = []  # Contains the results of the dijkstra (route, ttime, collmat)
    route = []  # Contains the vertexes which makes the route
    ttime = []  # Contains the travel time to each vertex
    collmat = []  # Contains the collision matrix

    timer_start = time.time()
    "Normal, consecutive version"
    return_idx = 0
    return_dict = {}
    for i in range(len(drone_v)):
        print("--------------------------------------------------------\nVel:", i)
        rtc = dijkstra_timed(G, V, mobs_obj, drone_v[i], i, return_dict)
        route.append(rtc[0])
        ttime.append(rtc[1])
        collmat.append(rtc[2])

    "multiprocessing.Pool - Queue TODO"
    # queue_dict = {}
    # queue_dict["input"] = {}
    # queue_dict["output"] = {}
    # target_function = dijkstra_timed
    # for i in range(len(drone_v)):
    #     input_values.append([G, V, mobs_obj, drone_v[i]])
    # # return_idx  = 0
    # return_dict = {}
    # jobs = []
    # for i in range(len(drone_v)):
    #     input_values_ = np.append(input_values[i], i)
    #     input_values_ = np.append(input_values_, return_dict)
    #     p = mp.Process(target=target_function,  args=input_values)
    #     jobs.append(p)
    #     p.start()

    # for proc in jobs:
    #     proc.join()

    # for i in range(len(drone_v)):
    #     rtc = return_dict[str(i)]
    #     route.append(rtc[0])
    #     ttime.append(rtc[1])
    #     collmat.append(rtc[2])

    timer_end = time.time()

    best = len(drone_v) - 1
    for i in range(len(drone_v)):  # Chose the velocity with the fastest time
        if ttime[i][-1] < ttime[best][-1]:
            best = i

    print("--------------------------------------------------------",
          "\nChosen vel:", best,
          "\nRoute:\n", route[best],
          "\nTime of flight:\n", ttime[best],
          "\nCollmat:\n", collmat[best],
          "\nTime of graph routing:", time_graph_E - time_graph_S, "sec",
          "\nTime of dijkstra:", timer_end - timer_start, "sec")
    # ......................................................................................................................
    # plotting the best path
    x = []
    y = []
    g = []
    for point in route[best]:
        g.append(V[point])
    for point in g:
        x.append(point[0])
        y.append(point[1])
    plt.plot(x, y, 'red', lw=1.5, alpha=0.5)

    path = Path_creator(np.array(g), "norm")
    X_path, Y_path = path
    plt.plot(X_path, Y_path, 'red', lw=1.5, alpha=1)

    path_tck, path_u = Path_creator(np.array(g), "csnp")
    drone_obj.append(Drone(drone_r, drone_v[best], path_tck, path_u[-1], [], []))
    # ----------------------------------------------------------------------------------------------------------------------
    # GUROBI
    # plt.show()
    results = gurobi_planner(drone_obj, mobs_obj)

    # ......................................................................................................................
    # generating csv
    # Trajectory of the drone
    # Dont ever do this again...:D
    # tmp_obj = []
    # tmp_obj.append(drone_obj[0])
    # tmp_obj.append(drone_obj[0])
    # generating_traj_csv(tmp_obj, "vehicle", 1)
    generating_traj_csv(drone_obj, "vehicle", 0, 1)

    # Trajectory of the moving obstacles
    for i in range(len(mobs_obj)):
        # Generating csv if it is actually a mobs
        if mobs_obj[i].mode != 'gurobi':
            generating_traj_csv(mobs_obj, "mobs", i)
        # However, if it is a disquised drone, then save it as a drone traj.
        else:
            generating_traj_csv([mobs_obj[i]], "vehicle", 0, 0)

    # ----------------------------------------------------------------------------------------------------------------------
    # SIMULATE THE FLIGHT

    global sim
    if sim:
        animation = flight_sim(mobs_obj, drone_obj, 10)  # Animate the moving objects (mobs, drone)
        plt.show()
    else:
        sim = True  # 2. input: animation frame delay in milliseconds

    return drone_obj


# if __name__ == '__main__':
def run_main(select=False):
    os.system("rm drone_x0.pickle")
    os.system("rm predef_index.pickle")
    if os.path.exists("predef_index.pickle"):
        pickle_in = open("predef_index.pickle", "rb")  # Loading the layout
        predef_index = pickle.load(pickle_in)
        pickle_in.close()
    else:
        predef_index = 0

    ko = 0.75
    predef_drone1_xf = [[-ko, ko], [ko, -ko], [0, ko], [0, -ko], [-ko, 0.3], [0.3, 0]]
    predef_drone2_xf = [[-ko, -ko], [ko, ko], [0, -ko], [0, ko], [-ko, -0.3], [0.7, 0]]

    #predef_drone1_xf = [[0.3, 0]]
    #predef_drone2_xf = [[0.7, 0]]

    if predef_index > len(predef_drone1_xf):
        predef_index = 0

    scene, mobs_obj, V0 = main()

    # Assuming we have saved an x0 positions for the drones
    if (select == False or select == "go_to_flip") and os.path.exists("drone_x0.pickle"):
        pickle_in = open("drone_x0.pickle", "rb")  # Loading the layout
        x0_values = pickle.load(pickle_in)
        pickle_in.close()
        drone1_x0 = x0_values[0]  # .tolist()
        drone2_x0 = x0_values[1]  # .tolist()

    # If we want to select from the GUI
    elif select == True:

        v1, v2 = select_SE(np.array([[0, 0]]), scene.fobs, scene.box)
        drone1_x0 = v1[-2, :].tolist()
        drone1_xf = v1[-1, :].tolist()
        drone2_x0 = v2[-2, :].tolist()
        drone2_xf = v2[-1, :].tolist()

    # If we did not find the pickle file, we set these x0 values.
    # (or if select == None)
    else:
        drone1_x0 = [-1.0, 0.5]
        drone2_x0 = [-1.0, -0.5]

    # ------------------------
    # If we don't want to select a final position from the GUI, then choose a random value for the 1st drone.
    if select == False or select == None:
        # Random position
        _, drone1_xf = random_V0_generator(x_lim=scene.dims[:2], y_lim=scene.dims[2:], d_min=1.0, p0=drone1_x0,
                                           d_max=1.5)

        """
        "Random position for drone 1"
        pos_list = [[0.75, 0.75], [-0.75, -0.75], [0.75, -0.75], [-0.75, 0.75], [0, 0.75], [0.75, 0], [-0.75, 0],
                    [0, -0.75]]
        pos_list_truncated = []
        # Selecting for drone 1
        import random
        while True:
            return_tuple = random.sample(list(enumerate(pos_list)), 1)
            list_index = return_tuple[0][0]
            drone1_xf = return_tuple[0][1]

            # If we accept it, because it is not the same as the starting position
            if np.linalg.norm(np.array(drone1_x0) - np.array(drone1_xf)) > 0.01:
                pos_list.pop(list_index)
                break
        """
        # Selecting from the predefined positions
        drone1_xf = predef_drone1_xf[predef_index]

        V0 = np.vstack((drone1_x0, drone1_xf))
    # If we want to go to the flip position
    elif select == "go_to_flip":
        drone1_xf = [0.8, 0.0]
        V0 = np.vstack((drone1_x0, drone1_xf))
    else:
        V0 = np.vstack((drone1_x0, drone1_xf))

    write_initial_position_yaml(drone1_x0, drone2_x0)

    drone_obj = main2(scene, mobs_obj, V0)
    mobs_obj.append(Mobs(0.08 * 1.5, "", drone_obj[0].path, "", "gurobi"))
    # mobs_obj.append(Mobs(0.08 * 0.1, "", drone_obj[0].path, "", "gurobi"))
    mobs_obj[-1].gp = drone_obj[0].gp
    mobs_obj[-1].gt = drone_obj[0].gt

    # If we don't want to select a final position from the GUI, then choose a random value for the 2nd drone.
    if select == False or select == None:
        _, drone2_xf = random_V0_generator(x_lim=scene.dims[:2], y_lim=scene.dims[2:], d_min=1.0, p0=drone2_x0,
                                           d_max=1.5, p2_clear=drone1_xf)

        """
        "Random position for drone 2"
        # Selecting for drone 2
        while True:
            return_tuple = random.sample(list(enumerate(pos_list)), 1)
            list_index = return_tuple[0][0]
            drone2_xf = return_tuple[0][1]

            # If we accept it, because it is not the same as the starting position
            if np.linalg.norm(np.array(drone2_x0) - np.array(drone2_xf)) > 0.01:
                pos_list.pop(list_index)
                break

        """
        # Selecting from the predefined positions
        drone2_xf = predef_drone2_xf[predef_index]

        V0 = np.vstack((drone2_x0, drone2_xf))
    # If we want to go to the flip position
    elif select == "go_to_flip":
        drone2_xf = [1.2, 0.0]
        V0 = np.vstack((drone2_x0, drone2_xf))
    else:
        V0 = np.vstack((drone2_x0, drone2_xf))

    drone_obj = main2(scene, mobs_obj, V0)

    # Let us save the final positions as x0 values for the next iteration
    pickle_out = open("drone_x0.pickle", "wb")  # Save scene
    pickle.dump([drone1_xf, drone2_xf], pickle_out)
    pickle_out.close()

    pickle_out = open("predef_index.pickle", "wb")  # Save scene
    pickle.dump(predef_index + 1, pickle_out)
    pickle_out.close()


# Options for select:
# True: we want to select x0 and xf values using the GUI
# False: we want random selection for the xf and we will read in the starting position
# from the pickle file. If the file doesn't exist, the we will use a pre-selected x0 values for the drones.
# "go_to_flip": in this case we will plan from the x0 (collected from the pickle file) to a predefined xf.
# This xf will be the starting positon of the flip

"""
predef_index = 1
ko = 0.75
predef_drone1_xf = [[-ko, ko], [ko, -ko], [0, ko], [0, -ko], [-ko, 0.3], [0.3, 0]]
predef_drone2_xf = [[-ko, -ko], [ko, ko], [0, -ko], [0, ko], [-ko, -0.3], [0.7, 0]]

pickle_out = open("drone_x0.pickle", "wb")  # Save scene
pickle.dump([predef_drone1_xf[predef_index], predef_drone2_xf[predef_index]], pickle_out)
pickle_out.close()

pickle_out = open("predef_index.pickle", "wb")  # Save scene
pickle.dump(predef_index + 1, pickle_out)
pickle_out.close()
"""

run_main(select = False)


"""
create_view(scene, idx)     
generating_traj_csv(drone_obj, "vehicle", 0)

for i in range(len(mobs_obj)):
    # Generating csv if it is actually a mobs
    if mobs_obj[i].mode != 'gurobi':
        generating_traj_csv(mobs_obj, "mobs", i)
    # However, if it is a disquised drone, then save it as a drone traj.
    else:
        # TODO: properly... this part is just ridiculous. But time is ticking...
        tmp_obj = mobs_obj[i]
        tmp_obj = [tmp_obj for j in range(len(mobs_obj))]
        generating_traj_csv(tmp_obj, "vehicle", 1)

create_view(scene, [6, 1, 2])             
animation = flight_sim(mobs_obj, drone_obj, 10)


If we like it, we can save the movement of the drone
pickle_out = open("drone_obj.pickle", "wb")     # Save scene
pickle.dump([drone_obj[0].gp, drone_obj[0].gt, drone_obj[0].path], pickle_out)
pickle_out.close()
"""

