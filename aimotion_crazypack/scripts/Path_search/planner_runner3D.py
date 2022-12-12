# This code should be run in a terminal window.
# Its job is to create new trajectories, if the
# do_optimization value is True
# This code is used alongside the dijkstra_simulation3D or 
# dijkstra_flight3D. These codes read the trajectories if the value 
# file_update is True and start the simulation/flight.
import time
from planner_tools_3D import pickle_write, pickle_read
import os


# How to run this on the laptop?
# cd to the folder of this file and then:
# chmod +x planner_runner3D.py
# /usr/bin/env python3.8 planner_runner3D.py



pickle_write("do_optimization", True)
pickle_write("file_updated", False)
pickle_write("predef_index", 0)
pickle_write("takeoff", False)
pickle_write("land", False)

if __name__ == '__main__':
    
    timer_start = time.time()
    cwd = os.getcwd()
    # Runs for 1000 seconds (circa... something here is wrong...)
    while(time.time() -  timer_start < 1000):
        do_optimization = pickle_read("do_optimization")
        if do_optimization == True:
            pickle_write("do_optimization", False)
            print("optimization in progress :)")
            exec(open(cwd + "/path_planner_3D.py").read())
            pickle_write("file_updated", True)
            pickle_write("takeoff", True) # takeoff is allows, because wa have new trajectories
            print("csv files are updated, you may go ahead and use them")
        else:
            time.sleep(1)
            # print("sleep for 0.1 seconds")
            print(time.time() -  timer_start)
            