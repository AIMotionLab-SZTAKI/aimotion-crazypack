import socket
import numpy as np
from pycrazyswarm import *

HOST = "192.168.1.248"  # The server's hostname or IP address
PORT = 1234  # The port used by the server


def tcp_data(data):
    orientation = None
    data_str = data.decode()
    x, y, z = data_str.split(', ')
    try:
        x = float(x)
        y = float(y)
        z = float(z)
    except:
        return orientation

    orientation = [x, y, z]
    return orientation


def fly():
    initHeight = 0.8
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # TakeOff
    allcfs.crazyflies[0].takeoff(targetHeight=initHeight, duration=3)
    timeHelper.sleep(1)
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        s.settimeout(15)
        orientation = []
        while orientation is not None:
            data = s.recv(15)
            orientation = tcp_data(data)
            if orientation[2] != 0:
                if orientation[2] > 0.45:
                    allcfs.crazyflies[0].cmdPosition(np.array([orientation[0], orientation[1], orientation[2]]), 0)
                else:
                    allcfs.crazyflies[0].cmdPosition(np.array([orientation[0], orientation[1], 0.45]), 0)

            else:
                timeHelper.sleep(1)
                allcfs.crazyflies[0].cmdStop()
                s.close()
        exit()
