import copy

import motioncapture
import zmq
import pickle
import time
import numpy as np

# Connect to optitrack
mc = motioncapture.MotionCaptureOptitrack("192.168.1.141")

# Create server
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.bind("tcp://*:50165")

t1 = copy.deepcopy(time.time())
drone_pos = [[],[],[],[]]
y_e = 1
m_idx = 0
T_m = copy.deepcopy(t1)

while True:
    mc.waitForNextFrame()
    t2 = copy.deepcopy(time.time())
    T_d = t2 - T_m  # time of flight between 2 points
    #print(t2-t1) # Measurement freqency
    #t1 = t2
    for name, obj in mc.rigidBodies.items():
        if name == 'bu13':  # The thing which triggers the movement
            #print(name, obj.position, obj.rotation.z) # The thing is measured by the optitrack?

            # For the timing of the movement
            pos_message = "{:.5f}".format(obj.position[0]) + "," + "{:.5f}".format(obj.position[1]) + "," + \
                          "{:.5f}".format(obj.position[2])
            socket.send(pos_message.encode())
            recived = socket.recv()
            """
            # For measurement
            if obj.position[1] > 0 > y_e:
                print(drone_pos[0][1], drone_pos[0][-1])
                print("New measurement")
                print("Drone_positions" + "{:.1f}".format(m_idx) + ".pickle")
                pickle_out = open("Drone_positions" + "{:.5f}".format(m_idx) + ".pickle", "wb")
                pickle.dump(drone_pos, pickle_out)
                pickle_out.close()
                drone_pos = [[],[],[],[]]
                m_idx = m_idx + 1
                T_m = copy.deepcopy(time.time())
                #print("New time", T_m)
            y_e = obj.position[1]
        if name == 'cf3':
            drone_pos[0].append([T_d, obj.position[0], obj.position[1], obj.position[2]])
        if name == 'cf5':
            drone_pos[1].append([T_d, obj.position[0], obj.position[1], obj.position[2]])
        if name == 'cf6':
            drone_pos[2].append([T_d, obj.position[0], obj.position[1], obj.position[2]])
        if name == 'cf10':
            drone_pos[3].append([T_d, obj.position[0], obj.position[1], obj.position[2]])
       
        if name == 'cf3':
            print(name, obj.position, obj.rotation.z)
            pos_message = "{:.5f}".format(obj.position[0]) + "," + "{:.5f}".format(obj.position[1]) + "," + \
                          "{:.5f}".format(obj.position[2])
            #socket.send(pos_message.encode())
            #recived = socket.recv()
            if obj.position[1] > 0 > y_e:
                print("New measurement")
                pickle_out = open("Drone_positions_all.pickle", "wb")
                pickle.dump(drone_pos, pickle_out)
                pickle_out.close()
                y_e = obj.position[1]
                #break
            y_e = obj.position[1]
            """


            #t2 = time.time()-t1
            #print("--------------------")
            #print("Name:", name)
            #print("Time:", t2, "sec")
            #print("Position:", obj.position)
            #print("Rotation:", obj.rotation.x, obj.rotation.y, obj.rotation.z, obj.rotation.w)
            #print("Distance:", float(recived))




