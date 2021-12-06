import numpy as np
from matplotlib import pyplot as plt
import math
import os
import csv
cwd = os.getcwd()

plt.close('all')
n_points = 100
acceleration_rounds = 0.5
acceleration_multiplier = 2
acceleration_multiplier = 2.5
i_step = 0.1 # this changes the resolution
radious = 0.8
cfID = 2
cfshift = 2 * math.pi / 3 * cfID


    
    


# initial values
phi0 = 0
v0 = 1

h = 0.7




"YAML"
import yaml
yaml_dict = {'crazyflies' : []}
for i in range(3):
    initialPosition = [float(radious * np.cos(phi0 +  2 * math.pi / 3 * i)), float(radious * np.sin(phi0 +  2 * math.pi / 3 * i))]
    initialPosition = initialPosition + [float(h)]
    yaml_dict['crazyflies'] += [{'id' : i, 'channel' : 100,
                                 'initialPosition' : initialPosition,
                                 'type' : 'default'
                                 }]
with open(cwd + "/yaml/initialPosition.yaml", "w") as file_descriptor:
    yaml.dump(yaml_dict, file_descriptor)
    
    
    

# For storing the polinomes
poly7_x = []
poly7_y = []
    

# Step 1: creating acceleration half-circle
phi = []
phi += [0 + cfshift]
i = 0
while phi[-1] < math.pi + cfshift:
    # if not the first step, then velocity is calculated from previous position
    if len(phi) > 1:
        v = (phi[-1] - phi[-2]) / i_step
    # if first step, velocity is v0
    else:
        v = v0
    # increasing phi given previous phi, current velocity and acceleration
    phi += [1/2 * acceleration_multiplier * i_step**2+ v * i_step + phi[-1]]
    i += i_step
    t1 = i
T = [i]
        
 

# compressing into a range of a half-circle
phi = np.array(phi)
shift = phi[0]
phi = phi - shift  # shifting it to start from zero
phi = phi / phi[-1] * (math.pi) 
phi = phi + shift


# creating x, y coordinates from it
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
plt.plot(x, y, 'k.')


                    

# fitting a polinome (with time running 0..t1 with resolution of i_step)
t = np.linspace(0, t1 , len(x))
poly7_x = [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y = [np.poly1d(np.polyfit(t, y, deg=7))]


# Step 2 acceleration half-circle 2
# setting up initial values
s0 = phi[-1]
v0 = (phi[-1] - phi[-2]) / i_step
# a0 = (phi[-1] - phi[-2]) / i_step**2

# creating a list of phi
phi = []
phi += [s0]
i = i_step
while phi[-1] < 2 * math.pi + cfshift:
    if len(phi) > 1:
        v = (phi[-1] - phi[-2]) / i_step
    else:
        v = v0
    phi += [1/2 * acceleration_multiplier * i_step**2+ v * i_step + phi[-1]]
    i += i_step
    t2 = i
T += [i]
    

# compressing into a range of a half-circle
phi = np.array(phi)
shift = phi[0]
phi = phi - shift  # shifting it to start from zero
phi = phi / phi[-1] * (math.pi) # compression
phi = phi + shift #shift back

# creating x, y coordinates
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
plt.plot(x, y, 'k.')

# fitting the polynome
t = np.linspace(0, t2, len(x))
poly7_x += [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y += [np.poly1d(np.polyfit(t, y, deg=7))]


    

    
    
# Step 3: traversing with constant speed
s0 = phi[-1]
v0 = (phi[-1] - phi[-2]) / i_step
# a0 = (phi[-1] - phi[-2]) / i_step**2
    
phi = []
phi += [0 + cfshift]
i = i_step
while phi[-1] < 2 * math.pi + cfshift:
    v = v0
    phi += [v * i_step + phi[-1]]
    i += i_step
    t3 = i
T += [i]

# compressing into a full circle
phi = np.array(phi)
shift = phi[0]
phi = phi - shift  # shifting it to start from zero
phi = phi / phi[-1] * (2*math.pi)
phi = phi + shift


# creating x, y coordinates
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
# plt.plot(x, y, 'k.')

# fitting the polynome
t = np.linspace(0, t3, len(x))
poly7_x += [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y += [np.poly1d(np.polyfit(t, y, deg=7))]


# Evaluating the polinome
# plt.figure()
# for i in range(2, len(poly7_x)):
#     # t = np.linspace(0.01, 1-0.01, 100)
#     t = np.linspace(0, 1, 100)
#     poly7_x_t = [poly7_x[i](t_) for t_ in t]
#     poly7_y_t = [poly7_y[i](t_) for t_ in t]
#     plt.plot(poly7_x_t, poly7_y_t)
#     # plt.plot(poly7_x_t, poly7_y_t, 'k.')


# T = [t1, t2, t3]


# repeat full-speed circle n times
n = 4 - 1
for i in range(n):
    T += [T[-1]]
    poly7_x += [poly7_x[-1]]
    poly7_y += [poly7_y[-1]]


    
# Step 4: deceleration half-circle
# setting up initial values
s0 = phi[-1]
v0 = (phi[-1] - phi[-2]) / i_step
# a0 = (phi[-1] - phi[-2]) / i_step**2

# creating a list of phi
phi = []
phi += [0 + cfshift] # this should be 0...
i = i_step

while phi[-1] < math.pi + cfshift:
    if len(phi) > 1:
        v = (phi[-1] - phi[-2]) / i_step
    else:
        v = v0
    phi += [1/2 * -0.5 * acceleration_multiplier * i_step**2+ v * i_step + phi[-1]]
    i += i_step
    t4 = i
T += [i]


# compressing into a range of a half-circle
phi = np.array(phi)
shift = phi[0] # this should be 0
phi = phi - shift  # shifting it to start from zero
phi = phi / phi[-1] * (math.pi) # compression
phi = phi + shift #shift back

# creating x, y coordinates from it
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
plt.plot(x, y, 'k.')

# fitting a polinome (with time running 0..t4 with resolution of i_step)
t = np.linspace(0, t4 , len(x))
poly7_x += [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y += [np.poly1d(np.polyfit(t, y, deg=7))]

# Step 5: another deceleration half-curve
# setting up initial values
s0 = phi[-1]
v0 = (phi[-1] - phi[-2]) / i_step
# a0 = (phi[-1] - phi[-2]) / i_step**2

# creating a list of phi
phi = []
phi += [s0] # this should be 0...
i = i_step

while phi[-1] < 2 * math.pi + cfshift:
    if len(phi) > 1:
        v = (phi[-1] - phi[-2]) / i_step
    else:
        v = v0
    phi += [1/2 * -0.5 * acceleration_multiplier * i_step**2+ v * i_step + phi[-1]]
    i += i_step
    t5 = i
T += [i]


# compressing into a range of a half-circle
phi = np.array(phi)
shift = phi[0] # this should be 0
phi = phi - shift  # shifting it to start from zero
phi = phi / phi[-1] * (math.pi) # compression
phi = phi + shift #shift back

# creating x, y coordinates from it
x = np.cos(phi.tolist()) * radious
y = np.sin(phi.tolist()) * radious
plt.plot(x, y)
plt.plot(x, y, 'k.')

# fitting a polinome (with time running 0..t5 with resolution of i_step)
t = np.linspace(0, t5 , len(x))
poly7_x += [np.poly1d(np.polyfit(t, x, deg=7))]
poly7_y += [np.poly1d(np.polyfit(t, y, deg=7))]



    
################################################################################################
# poly7_z

poly7_z = []
for i in range(len(T)):
    t = np.linspace(0, T[i], 100)
    if i > 1 and i < len(T) - 2:
        poly7_z_t = [np.sin(1 * (2 * math.pi) * t_ / T[i]) * 0.1 for t_ in t]
    else:
        poly7_z_t = [0.01 for t_ in t]
    poly7_z += [np.poly1d(np.polyfit(t, poly7_z_t, deg=7))]
    plt.figure(4)
    plt.plot(t, poly7_z_t)
    plt.plot(t, [poly7_z[i](t_) for t_ in t], '*')
################################################################################################


################################################################################################
# poly7_j

poly7_j = []
for i in range(len(T)):
    t = np.linspace(0, T[i], 100)
    poly7_x_t = [poly7_x[i](t_) for t_ in t]
    poly7_y_t = [poly7_y[i](t_) for t_ in t]
    plt.figure(1)
    plt.plot(poly7_x_t, poly7_y_t)
    plt.figure(2)
    t = np.linspace(np.sum(T[:i]), np.sum(T[:i+1]), 100)
    plt.plot(t, poly7_x_t)
    plt.plot(t, poly7_y_t)
    plt.figure(3)
    # poly7_j_t = [math.atan(poly7_y_t_ / poly7_x_t_) for poly7_y_t_, poly7_x_t_ in zip(poly7_y_t, poly7_x_t)]
    
    """
    """
    if i > 1 and i < len(T) - 2:
        poly7_j_t = np.linspace(0 + cfshift, 2*math.pi + cfshift, 100)
    else:
        if i == 0 or i == len(T) - 2:
            poly7_j_t = np.linspace(0 + cfshift, math.pi + cfshift, 100)
        else:
            poly7_j_t = np.linspace(0 + cfshift + math.pi, 2 * math.pi + cfshift, 100)
        
    
    
    plt.plot(t, poly7_j_t, 'o')
    t = np.linspace(0, T[i], 100)
    poly7_j += [np.poly1d(np.polyfit(t, poly7_j_t, deg=7))]
    t = np.linspace(np.sum(T[:i]), np.sum(T[:i+1]), 100)
    plt.plot(t, [poly7_j[i](t) for t in np.linspace(0, T[i], 100)]   , '*')
################################################################################################      
    

    

first_line = ['duration', 'x^0', 'x^1', 'x^2', 'x^3', 'x^4', 'x^5', 'x^6', 'x^7', 'y^0', 'y^1', 'y^2', 'y^3', 'y^4', 'y^5', 'y^6', 'y^7', 'z^0', 'z^1', 'z^2', 'z^3', 'z^4', 'z^5', 'z^6', 'z^7', 'yaw^0', 'yaw^1', 'yaw^2', 'yaw^3', 'yaw^4', 'yaw^5', 'yaw^6', 'yaw^7']
mode = 'w'
with open(cwd + '/csv/vehicle' + str(cfID) + '.csv', mode = mode) as csvfile:
    writer = csv.writer(csvfile, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    writer.writerow(first_line)
    
    # i = 1
    # if i == 1:
    for i in range(len(T)):
        px = poly7_x[i].coeffs.tolist()
        px.reverse()
        py = poly7_y[i].coeffs.tolist()
        py.reverse()
        pz = poly7_z[i].coeffs.tolist()
        pz.reverse()
        pz = [0] * len(px)
        pj = poly7_j[i].coeffs.tolist()
        pj.reverse()
        # pj = [0] * len(px)
        writer.writerow([T[i]] + px + py + pz + pj)
        
        
        
  
        