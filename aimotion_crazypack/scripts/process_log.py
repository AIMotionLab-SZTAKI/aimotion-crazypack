#!/usr/bin/env python

import numpy as np
import rospy
from crazyswarm.msg import GenericLogData
from pycrazyswarm import *
import matplotlib.pyplot as plt

class DataProcessing:
    def __init__(self):
        self.values = []
        self.value = None
        self.times = []
        self.time = None

    def callback(self, data):
        self.value = data.values
        self.time = data.header.stamp.to_sec()
    def listener(self):
        rospy.init_node('listener')
        rospy.Subscriber("/cf4/log1", GenericLogData, self.callback)
        duration = 15
        dt = 0.01
        for _ in range(int(duration/dt)):
            if self.value is not None and self.time is not None:
                self.values = self.values + [list(self.value)]
                self.times = self.times + [self.time]
            rospy.sleep(dt)
        self.times = [time - self.times[0] for time in self.times]
        values = np.array(self.values)
        values = values.T
        self.values = values.tolist()
        fig, axs = plt.subplots(len(self.values))
        # fig, axs = plt.subplots(3)
        for i, log_var in enumerate(self.values):
            axs[i].plot(self.times, log_var)
            axs[i].grid(True)
        # axs[0].plot(self.times, self.values[0])
        # axs[0].plot(self.times, self.values[1])
        # axs[1].plot(self.times, self.values[2])
        # axs[1].plot(self.times, self.values[3])
        # axs[2].plot(self.times, self.values[4])
        # axs[2].plot(self.times, self.values[5])
        plt.show()

if __name__ == "__main__":
    # swarm = Crazyswarm()
    # timeHelper = swarm.timeHelper
    # allcfs = swarm.allcfs

    DP = DataProcessing()
    DP.listener()
