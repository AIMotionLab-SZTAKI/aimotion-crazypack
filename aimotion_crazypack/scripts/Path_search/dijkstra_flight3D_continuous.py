#!/usr/bin/env python
from pycrazyswarm import *
import uav_trajectory
import argparse
import yaml
import os
from os.path import exists
import copy
import time
import zmq
import numpy as np

# -- Because the order matters --
# Running crazyswarm
swarm = Crazyswarm(crazyflies_yaml="../../launch/crazyflies.yaml")
timeHelper = swarm.timeHelper
allcfs = swarm.allcfs

# Setting parameter
#colors = [[100, 0, 0], [0, 0, 100], [0, 100, 0], [100, 100, 100]]
#clr_i = 0
for cf in allcfs.crazyflies:
    cf.setParam('stabilizer/controller', 1)
    #cf.setParam('ring/effect', 7)
    #cf.setparam('ring/solidRed', '100')    # R
    #cf.setparam('ring/solidGreen', '0')  # G
    #cf.setparam('ring/solidBlue', '0')   # B
    #clr_i = clr_i+1

import pickle
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


# Connect to server
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.connect("tcp://localhost:50165")
mark = np.array([[-0.9, 0], [1.3, -1.4]])
mark_time = copy.deepcopy(time.time())
mark_i = 0
eps = 0.2

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # parser.add_argument('--takeoff', default=True, type=bool)
    # parser.add_argument('--land', default=True, type=bool)
    parser.add_argument('--max_drones', default=4, type=int)
    parser.add_argument('--zoffset', default=0.0, type=float)
    ARGS = parser.parse_args()
        
    # Waiting for takeoff command
    i = 0
    while(pickle_read("takeoff") == False):
        print("Waiting for takeoff command")
        time.sleep(1)
        i += 1
        if i == 60:
            print("Flight aborted because no takeoff command was received")
            assert 0, "Out of time"
            
    pickle_write("takeoff", False)
    #############
    # Takeoff!! #
    #############
    # Reading initial positions
    cwd = os.getcwd()
    cwd = cwd
    with open(cwd + "/dijkstra3D/yaml/initialPosition.yaml", "r") as file:
        initialPosition = yaml.load(file, Loader=yaml.FullLoader)

    
    # Take off
    for j, cf in enumerate(allcfs.crazyflies):
        pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
        cf.takeoff(targetHeight=pos[2] + j * ARGS.zoffset - ARGS.zoffset, duration=3)
    timeHelper.sleep(3.0)

    # Go to initial position
    for j, cf in enumerate(allcfs.crazyflies):
        pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
        cf.goTo(pos + np.array([0, 0, j * ARGS.zoffset - ARGS.zoffset]), 0, 2.0)
    timeHelper.sleep(2)
    
    ############
    # Fly-loop #
    ############
    # Flying, until we are asked to land.
    while True:
        
        # Waiting until the files are updated
        while(pickle_read("file_updated") == False):
            print("Waiting for updated trajectories")
            time.sleep(0.1)

        # SAFETY CHECK
        # print("Good Lord, trajectories are updated. Can I read them?")
        # swarm.input.waitUntilButtonPressed()
        

        # Loading trajectories
        trajectories = []
        for i in range(ARGS.max_drones):
            if exists(cwd + "/dijkstra3D/csv/vehicle" + str(i) + ".csv"):
                traj = uav_trajectory.Trajectory()
                traj.loadcsv(cwd + "/dijkstra3D/csv/vehicle" + str(i) + ".csv")
                trajectories.append(copy.deepcopy(traj))
            else:
                print("vehicle" + str(i) +".csv" + " doesn't exist")
        n_drones = len(trajectories)

        # SAFETY CHECK
        # print("Trajectories have been read. Can I upload the trajectory?")
        # swarm.input.waitUntilButtonPressed()

        # Important: we need to use the goTo command before the upload trajectory,
        # otherwise everything goes haywire
        # The goTo position can be read from the yaml file, which should also be updated by this time.
        # It only needs 0.1 seconds to go there, because it should already be there anyways...
        with open(cwd + "/dijkstra3D/yaml/initialPosition.yaml", "r") as file:
            initialPosition = yaml.load(file, Loader=yaml.FullLoader)
        for j, cf in enumerate(allcfs.crazyflies):
            pos = np.array(initialPosition['crazyflies'][j]['initialPosition'])
            cf.goTo(pos, 0, 0.1)
        timeHelper.sleep(0.1)

        # --> now we can upload the trajectory :)
        # Upload trajectories to cf
        for j, cf in enumerate(allcfs.crazyflies):
            cf.uploadTrajectory(0, 0, trajectories[j])

        # SAFETY CHECK
        # print("Trajectories have been uploaded. Can I start following the trajectory?")
        # swarm.input.waitUntilButtonPressed()
        y_e = 1
        while True:
            message = socket.recv()
            message = str(message)
            pos = list(message[2:-1].split(","))
            pos = np.array([float(pos[0]), float(pos[1]), float(pos[2])])

            dist = np.linalg.norm(mark[0] - pos[0:2])
            dist_message = "{:.5f}".format(dist)
            socket.send(dist_message.encode())

            if pos[1] > 0 > y_e:
                timescale = 1.0
                allcfs.startTrajectory(0, timescale=timescale, reverse=False)
                print("START")
                y_e = pos[1]
                break
            y_e = pos[1]
        
        # All trajectories are read, let us now allow the next optimization to happen
        pickle_write("file_updated", False)
        pickle_write("do_optimization", True)
        
        
        # Waiting until all of them finish
        max_traj_time = 0
        for j, cf in enumerate(allcfs.crazyflies):
            print("Drone " + str(j) + " time is: " + str(trajectories[j].duration) + "s")
            max_traj_length = max(max_traj_time, trajectories[j].duration)
        print("Waiting " + str(max_traj_length) + "s until all trajectories are finished")
        timeHelper.sleep(max_traj_length * timescale + 0.0)


        # TODO: evading trajectories will increase the needed waiting time

        # SAFETY CHECK
        # print("Done following the trajectories... Can I do the optimization?")
        # swarm.input.waitUntilButtonPressed()

    
        if pickle_read("land") == True:
            break
        
    ###########
    # Landing #
    ###########
    allcfs.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(2.0)

