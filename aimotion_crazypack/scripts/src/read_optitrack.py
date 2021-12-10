import pandas
import numpy as np
# df = pandas.read_csv('optitrack.csv')
# df = np.array(df)

# rigid_bodies = {}
# data = np.array([])
# name = []
# for i in range(df.shape[1]):
#     if df.iloc[2, i] == "Rigid Body":
#         name += df[3, i]
#         data = np.append(np.array(df[7:, :]).reshape(-1, 1))


# from numpy import genfromtxt
# # my_data = genfromtxt('optitrack.csv', delimiter=',')
# data = np.loadtxt('optitrack.csv',delimiter=';',skiprows=0) 



# import csv
# import numpy as np
# import matplotlib.pyplot as plt

# data_path = 'optitrack.csv'
# data = np.array([])
# with open(data_path, 'r') as f:
#     reader = csv.reader(f, delimiter=',')
#     headers = next(reader)
#     # data = np.array(list(reader)).astype(float)
#     data = np.append(data,headers)

def read_optitrack(filename = './csv/optitrack.csv'):
    
    with open(filename,'r') as f:
        lines = f.readlines()[1:]
        
    lines2 = []
    for line in lines:
        lines2 += [line.split(',')]
        
    lines2 = lines2[1:]
    df = np.array(lines2)
    

    
    data_dictionary = {}
    for i in range(df.shape[1]):
        # If the first line is "Rigid Body"
        if df[0, i] == "Rigid Body":
            # If we have already dealt with this concrete rigid body
            if df[1, i] in data_dictionary:
                pass
            else:
                data = df[5:, i:i+6+1]
                np_data = np.array([])
                for j in range(data.shape[0]):
                    try:
                        # np_data = np.append(np_data, np.array(data[j, :], dtype = np.float64).reshape(1, -1))
                        if len(np_data) == 0:
                            np_data = np.array(data[j, :], dtype = np.float64).reshape(1, -1)
                        else:
                            np_data = np.vstack((np_data, np.array(data[j, :], dtype = np.float64).reshape(1, -1)))
                    except:
                        pass
                data_dictionary[df[1, i]] = np_data
    
    position_dictionary = {}
    for key in data_dictionary:
        position_dictionary[key] = {
                'x' : np.mean(data_dictionary[key][:, 4]),
                'y' : np.mean(data_dictionary[key][:, 5]),
                'z' : np.mean(data_dictionary[key][:, 6])
            }
    
    obstacle_positions = {}
    vehicle_positions = {}
    for data in position_dictionary:
        # if list(position_dictionary.items())[0][0][0:2] == "cf":
        #     vehicle_positions[data] = position_dictionary[data]
        if data[0:2] == "cf":
            vehicle_positions[data] = position_dictionary[data]
            
    for data in position_dictionary:
        if data[0:2] == "ob":
            obstacle_positions[data] = position_dictionary[data]
    
    
    
    return obstacle_positions, vehicle_positions

# obstacle_positions, vehicle_positions = read_optitrack()
# print(vehicle_positions["cf5"]["x"])