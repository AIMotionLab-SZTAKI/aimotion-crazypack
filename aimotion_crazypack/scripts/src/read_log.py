import numpy as np
import pandas

def read_log(filename = 'logcf5.csv'):
    
    
    df = pandas.read_csv(filename)
    statePos_x_tmp = np.array(df["ctrlMel.statePos_x"])
    statePos_y_tmp = np.array(df["ctrlMel.statePos_y"])
    statePos_z_tmp = np.array(df["ctrlMel.statePos_z"])
    setpointPos_x_tmp = np.array(df["ctrlMel.setpointPos_x"])
    setpointPos_y_tmp = np.array(df["ctrlMel.setpointPos_y"])
    setpointPos_z_tmp = np.array(df["ctrlMel.setpointPos_z"])
    
    
    statePos_x = []
    statePos_y = []
    statePos_z = []
    setpointPos_x = []
    setpointPos_y = []
    setpointPos_z = []
    
    log_data = np.array(df["ctrlMel.log_data"])
    
    for i in range(len(df.time)):
        if log_data[i] == 1:
            
            statePos_x += [statePos_x_tmp[i]]
            statePos_y += [statePos_y_tmp[i]]
            statePos_z += [statePos_z_tmp[i]]
            setpointPos_x += [setpointPos_x_tmp[i]]
            setpointPos_y += [setpointPos_y_tmp[i]]
            setpointPos_z += [setpointPos_z_tmp[i]]
    
            
    res = {}
    res["statePos_x"] = statePos_x
    res["statePos_y"] = statePos_y
    res["statePos_z"] = statePos_z
    res["setpointPos_x"] = setpointPos_x
    res["setpointPos_y"] = setpointPos_y
    res["setpointPos_z"] = setpointPos_z
    
    return res
        
res = read_log()

from matplotlib import pyplot as plt
plt.plot(res["statePos_x"])
plt.plot(res["setpointPos_x"])