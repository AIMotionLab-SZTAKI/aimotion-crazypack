import zmq
import numpy as np
from time import sleep
# Create server
context = zmq.Context()
socket = context.socket(zmq.REP)
socket.connect("tcp://localhost:50165")

mark = np.array([-0.9, 0])
eps = 0.1
y_e = -1

while True:
    message = socket.recv()

    message = str(message)
    pos = list(message[2:-1].split(","))
    pos = np.array([float(pos[0]), float(pos[1]), float(pos[2])])

    if pos[1] > 0 > y_e:
        print("START")
    y_e = pos[1]

    dist = np.linalg.norm(mark - pos[0:2])
    dist_message = "{:.5f}".format(dist)
    socket.send(dist_message.encode())
    #print(dist)
    #if dist <= eps:
    #    print("START_dist")
    #    print(dist)



